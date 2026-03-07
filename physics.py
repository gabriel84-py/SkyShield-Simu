"""
physics.py — Modèle physique du drone SkyShield

Paramètres tirés du vrai drone (notes.md) :
  - Masse        : 444g
  - Poussée max  : 1600g (4 moteurs)
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
THRUST_MAX_KG   = 1.600          # poussée totale max (kg-force) → 4 moteurs
THRUST_MAX_N    = THRUST_MAX_KG * GRAVITY   # en Newtons

ARM_LENGTH_M    = 0.125          # demi-diagonale frame 250mm (m)

# Moments d'inertie estimés (drone 250mm, géométrie X symétrique)
# I = m * r² approximatif pour une masse répartie sur les bras
# On affine pour que la réponse dynamique corresponde au CSV réel
INERTIA_ROLL    = 0.003          # kg·m² axe roll
INERTIA_PITCH   = 0.003          # kg·m² axe pitch (symétrique)

# Frottement aérodynamique angulaire (amorti naturel sans PID)
# Évite une oscillation infinie non physique
DRAG_ANGULAR    = 0.05           # N·m·s/rad

# Frottement aérodynamique vertical
DRAG_VERTICAL   = 0.8            # N·s/m

# Conversion throttle % → poussée
# À 100% : THRUST_MAX_N sur les 4 moteurs
# Chaque moteur contribue à THRUST_MAX_N / 4 à plein régime
THRUST_PER_MOTOR_MAX = THRUST_MAX_N / 4.0   # N par moteur à 100%

# Seuil moteur minimum (identique à flight_controller.py)
MIN_MOTOR = 10.0    # % — en dessous le moteur ne tourne pas


# ─────────────────────────────────────────────────────────────────
class DronePhysics:
    """
    Modèle physique complet du drone SkyShield.

    État interne :
      roll, pitch          → angles (degrés)
      roll_rate, pitch_rate → vitesses angulaires (degrés/s)
      altitude             → hauteur (m)
      vz                   → vitesse verticale (m/s)

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

        # ── perturbation externe (couple impulsionnel) ────────────
        self._perturb_roll  = 0.0    # N·m, consommé en 1 step
        self._perturb_pitch = 0.0

        # ── sol : le drone ne peut pas descendre sous 0 ──────────
        self.on_ground  = True

    # ─────────────────────────────────────────────────────────────
    def _motor_thrust(self, throttle_pct: float) -> float:
        """
        Convertit un throttle en % → force en Newtons pour UN moteur.
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

        Moteurs (config X, vue de dessus) :
          M1 front-right (CW)   M2 front-left (CCW)
          M4 rear-right  (CCW)  M3 rear-left  (CW)

        Couples :
          Roll  = (M2 + M3 - M1 - M4) * ARM_LENGTH  (gauche+ = roll+)
          Pitch = (M1 + M2 - M3 - M4) * ARM_LENGTH  (avant+  = pitch+)
        """
        # ── forces de poussée par moteur ─────────────────────────
        f1 = self._motor_thrust(m1)
        f2 = self._motor_thrust(m2)
        f3 = self._motor_thrust(m3)
        f4 = self._motor_thrust(m4)

        total_thrust = f1 + f2 + f3 + f4

        # ── couples angulaires (N·m) ─────────────────────────────
        torque_roll  = (f2 + f3 - f1 - f4) * ARM_LENGTH_M
        torque_pitch = (f1 + f2 - f3 - f4) * ARM_LENGTH_M

        # ── ajout perturbation externe ────────────────────────────
        torque_roll  += self._perturb_roll
        torque_pitch += self._perturb_pitch
        self._perturb_roll  = 0.0   # consommé
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
        # On projette la poussée selon l'inclinaison réelle du drone
        tilt_factor = math.cos(math.radians(self.roll)) * math.cos(math.radians(self.pitch))
        tilt_factor = max(0.0, tilt_factor)  # ne pousse jamais vers le bas

        weight   = MASS_KG * GRAVITY
        fz_net   = total_thrust * tilt_factor - weight - DRAG_VERTICAL * self.vz

        az = fz_net / MASS_KG
        self.vz       += az * dt
        self.altitude += self.vz * dt

        # ── sol ───────────────────────────────────────────────────
        if self.altitude <= 0.0:
            self.altitude  = 0.0
            self.vz        = max(0.0, self.vz)   # rebond absorbé
            self.on_ground = (total_thrust < weight * 0.95)
        else:
            self.on_ground = False

    # ─────────────────────────────────────────────────────────────
    def apply_perturbation(self, roll_deg: float = 0.0, pitch_deg: float = 0.0):
        """
        Applique un coup de vent impulsionnel.
        roll_deg / pitch_deg : amplitude en degrés/s² équivalent
        Converti en couple N·m pour rester cohérent avec les unités.
        """
        # On injecte directement dans la vitesse angulaire pour un effet immédiat
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