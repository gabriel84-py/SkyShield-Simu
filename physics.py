"""
physics.py — Modèle physique du drone SkyShield

Paramètres tirés du vrai drone (notes.md) :
  - Masse        : 444g
  - Poussée max  : ~2470g (4 moteurs) — calibré pour hover à ~18% throttle
  - Frame        : 250mm True X
  - Batterie     : 3S 11.45V
  - Config       : X (M1 front-right, M2 front-left, M3 rear-left, M4 rear-right)

Modèle 3 axes : roll, pitch, altitude
Intégration Euler à pas fixe (dt = 0.01s → 100 Hz, comme le vrai drone)

Repère :
  - Roll  > 0 → drone penche à droite
  - Pitch > 0 → drone penche en avant
  - Z     > 0 → drone monte

Mixage moteurs (identique à flight_controller.py) :
  M1 = throttle - corr_roll - corr_pitch   (front-right)
  M2 = throttle + corr_roll - corr_pitch   (front-left)
  M3 = throttle + corr_roll + corr_pitch   (rear-left)
  M4 = throttle - corr_roll + corr_pitch   (rear-right)
"""

import math


# ─────────────────────────────────────────────────────────────────
# Constantes physiques du drone réel
# ─────────────────────────────────────────────────────────────────

MASS_KG         = 0.444          # masse totale (kg)
GRAVITY         = 9.81           # m/s²

# Poussée totale max calibrée pour un hover à ~18% throttle.
# Ratio thrust/weight ≈ 2.5 — typique pour un 250mm en banc de test contrôlé.
# (Le vrai drone peut monter plus haut mais le banc limite à 30% throttle)
THRUST_MAX_N    = MASS_KG * GRAVITY / 0.18 * (30.0 / 100.0)  # ≈ 7.26 N à 30%

ARM_LENGTH_M    = 0.125          # demi-diagonale frame 250mm (m)

# Moments d'inertie estimés (drone 250mm, géométrie X symétrique)
INERTIA_ROLL    = 0.003          # kg·m² axe roll
INERTIA_PITCH   = 0.003          # kg·m² axe pitch (symétrique)

# Frottement aérodynamique angulaire — maintenu faible pour que
# les oscillations PID soient visibles à l'écran
DRAG_ANGULAR    = 0.05           # N·m·s/rad

# Frottement aérodynamique vertical
DRAG_VERTICAL   = 0.8            # N·s/m

# Conversion throttle % → poussée par moteur
THRUST_PER_MOTOR_MAX = THRUST_MAX_N / 4.0   # N par moteur à 100%

# Constante de temps de réponse des moteurs (s).
# Modélise le délai ESC + montée en régime (réaliste : 50–100ms).
# C'est CE délai qui permet au PID oscillant (Kd=0) de produire
# des oscillations visibles à l'écran.
MOTOR_TAU       = 0.07           # secondes

# Seuil moteur minimum (identique à flight_controller.py)
MIN_MOTOR = 10.0    # % — en dessous le moteur ne tourne pas

# Hover throttle théorique (utile pour référence dans les commentaires)
# hover_throttle = MASS_KG * GRAVITY / THRUST_MAX_N * 100 ≈ 18 %


# ─────────────────────────────────────────────────────────────────
class DronePhysics:
    """
    Modèle physique complet du drone SkyShield.

    État interne :
      roll, pitch          → angles (degrés)
      roll_rate, pitch_rate → vitesses angulaires (degrés/s)
      altitude             → hauteur (m)
      vz                   → vitesse verticale (m/s)

    Nouveauté : les throttles moteurs passent par un filtre passe-bas
    (constante de temps MOTOR_TAU) qui modélise le délai de réponse
    réel des ESC/moteurs. Sans ce délai, le PID corrige en 1 step
    et aucune oscillation n'est visible même avec Kd=0.

    Interface :
      step(m1, m2, m3, m4, dt)  → applique les throttles moteurs, intègre dt
      apply_perturbation(...)   → simule un coup de vent
      reset()                   → remet tout à zéro
    """

    def __init__(self):
        # ── angles (degrés) ──────────────────────────────────────
        self.roll       = 0.0
        self.pitch      = 0.0

        # ── vitesses angulaires (degrés/s) ───────────────────────
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0

        # ── altitude (m) et vitesse verticale (m/s) ──────────────
        self.altitude   = 0.0
        self.vz         = 0.0

        # ── état interne moteurs (filtre passe-bas) ───────────────
        # Initialisés à 0 ; montée progressive au premier armement
        self._m1 = 0.0
        self._m2 = 0.0
        self._m3 = 0.0
        self._m4 = 0.0

        # ── perturbation externe (couple impulsionnel) ────────────
        self._perturb_roll  = 0.0
        self._perturb_pitch = 0.0

        # ── sol : le drone ne peut pas descendre sous 0 ──────────
        self.on_ground  = True

    # ─────────────────────────────────────────────────────────────
    def _motor_thrust(self, throttle_pct: float) -> float:
        """
        Convertit un throttle filtré en % → force en Newtons pour UN moteur.
        En dessous de MIN_MOTOR : moteur silencieux → 0 N.
        Relation linéaire (approximation raisonnable pour un banc de test).
        """
        if throttle_pct < MIN_MOTOR:
            return 0.0
        t = max(0.0, min(100.0, throttle_pct))
        return (t / 100.0) * THRUST_PER_MOTOR_MAX

    # ─────────────────────────────────────────────────────────────
    def step(self, m1: float, m2: float, m3: float, m4: float, dt: float):
        """
        Intègre la physique sur un pas de temps dt (secondes).

        Les throttles m1..m4 sont les CONSIGNES envoyées par le FC.
        Elles passent par un filtre passe-bas (MOTOR_TAU) avant d'être
        converties en forces — c'est le modèle de délai moteur.

        Moteurs (config X, vue de dessus) :
          M1 front-right (CW)   M2 front-left (CCW)
          M4 rear-right  (CCW)  M3 rear-left  (CW)

        Couples :
          Roll  = (M2 + M3 - M1 - M4) * ARM_LENGTH  (gauche+ = roll+)
          Pitch = (M1 + M2 - M3 - M4) * ARM_LENGTH  (avant+  = pitch+)
        """
        # ── filtre passe-bas moteurs ──────────────────────────────
        # alpha = dt / (dt + tau) ≈ dt/tau pour tau >> dt
        alpha = dt / (dt + MOTOR_TAU)
        self._m1 += alpha * (m1 - self._m1)
        self._m2 += alpha * (m2 - self._m2)
        self._m3 += alpha * (m3 - self._m3)
        self._m4 += alpha * (m4 - self._m4)

        # ── forces de poussée par moteur (throttles filtrés) ──────
        f1 = self._motor_thrust(self._m1)
        f2 = self._motor_thrust(self._m2)
        f3 = self._motor_thrust(self._m3)
        f4 = self._motor_thrust(self._m4)

        total_thrust = f1 + f2 + f3 + f4

        # ── couples angulaires (N·m) ─────────────────────────────
        torque_roll  = (f2 + f3 - f1 - f4) * ARM_LENGTH_M
        torque_pitch = (f1 + f2 - f3 - f4) * ARM_LENGTH_M

        # ── ajout perturbation externe ────────────────────────────
        torque_roll  += self._perturb_roll
        torque_pitch += self._perturb_pitch
        self._perturb_roll  = 0.0   # consommé en 1 step
        self._perturb_pitch = 0.0

        # ── dynamique angulaire (Newton) ──────────────────────────
        # α = (τ - drag * ω) / I
        roll_rate_rad  = math.radians(self.roll_rate)
        pitch_rate_rad = math.radians(self.pitch_rate)

        alpha_roll  = (torque_roll  - DRAG_ANGULAR * roll_rate_rad)  / INERTIA_ROLL
        alpha_pitch = (torque_pitch - DRAG_ANGULAR * pitch_rate_rad) / INERTIA_PITCH

        # intégration Euler vitesse angulaire (rad/s → deg/s)
        self.roll_rate  += math.degrees(alpha_roll)  * dt
        self.pitch_rate += math.degrees(alpha_pitch) * dt

        # intégration Euler angle
        self.roll  += self.roll_rate  * dt
        self.pitch += self.pitch_rate * dt

        # ── clamp angles (sécurité physique) ─────────────────────
        self.roll  = max(-180.0, min(180.0, self.roll))
        self.pitch = max(-180.0, min(180.0, self.pitch))

        # ── dynamique verticale ───────────────────────────────────
        # Force nette = poussée projetée verticalement - poids - frottement
        tilt_factor = math.cos(math.radians(self.roll)) * math.cos(math.radians(self.pitch))
        tilt_factor = max(0.0, tilt_factor)

        weight   = MASS_KG * GRAVITY
        fz_net   = total_thrust * tilt_factor - weight - DRAG_VERTICAL * self.vz

        az = fz_net / MASS_KG
        self.vz       += az * dt
        self.altitude += self.vz * dt

        # ── sol ───────────────────────────────────────────────────
        if self.altitude <= 0.0:
            self.altitude  = 0.0
            self.vz        = max(0.0, self.vz)
            self.on_ground = (total_thrust < weight * 0.95)
        else:
            self.on_ground = False

    # ─────────────────────────────────────────────────────────────
    def apply_perturbation(self, roll_deg: float = 0.0, pitch_deg: float = 0.0):
        """
        Applique un coup de vent impulsionnel.
        Injecte directement dans la vitesse angulaire pour un effet immédiat.
        """
        self.roll_rate  += roll_deg
        self.pitch_rate += pitch_deg

    # ─────────────────────────────────────────────────────────────
    def get_state(self) -> dict:
        """Retourne l'état complet du drone (utile pour le logger/visualizer)."""
        return {
            "roll":        self.roll,
            "pitch":       self.pitch,
            "roll_rate":   self.roll_rate,
            "pitch_rate":  self.pitch_rate,
            "altitude":    self.altitude,
            "vz":          self.vz,
            "on_ground":   self.on_ground,
        }

    # ─────────────────────────────────────────────────────────────
    def reset(self):
        """Remet le drone à l'état initial (posé à plat)."""
        self.roll       = 0.0
        self.pitch      = 0.0
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0
        self.altitude   = 0.0
        self.vz         = 0.0
        self.on_ground  = True
        self._perturb_roll  = 0.0
        self._perturb_pitch = 0.0
        # Reset des états moteurs filtrés
        self._m1 = 0.0
        self._m2 = 0.0
        self._m3 = 0.0
        self._m4 = 0.0