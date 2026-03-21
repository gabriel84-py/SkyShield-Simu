"""
-Gab
physics.py — Modèle physique du drone SkyShield

Paramètres tirés du vrai drone :
  - Masse        : 444g
  - Frame        : 250mm True X
  - Config       : X (M1 front-right, M2 front-left, M3 rear-left, M4 rear-right)

Calibration :
  hover_throttle ≈ 20% → THRUST_MAX_N = poids / 0.20
  (ratio thrust/weight = 5.0 à pleine puissance — typique pour un 250mm)

Modèle 3 axes : roll, pitch, altitude
Intégration Euler à pas fixe (dt = 0.01s → 100 Hz)

Repère :
  Roll  > 0 → drone penche à droite
  Pitch > 0 → drone penche en avant
  Z     > 0 → drone monte
"""

import math

# ─────────────────────────────────────────────────────────────────
# Constantes physiques
# ─────────────────────────────────────────────────────────────────

MASS_KG   = 0.444
GRAVITY   = 9.81
weight    = MASS_KG * GRAVITY          # 4.356 N

# Hover calibré à 20% throttle.
# Formule : THRUST_MAX = poids / (hover_pct / 100)
HOVER_PCT         = 20.0
THRUST_MAX_N      = weight / (HOVER_PCT / 100.0)   # 21.78 N
THRUST_PER_MOTOR  = THRUST_MAX_N / 4.0             #  5.44 N par moteur à 100%

ARM_LENGTH_M  = 0.125   # demi-diagonale 250mm (m)
INERTIA_ROLL  = 0.003   # kg·m²
INERTIA_PITCH = 0.003   # kg·m²
DRAG_ANGULAR  = 0.05    # N·m·s/rad  (amorti naturel)
DRAG_VERTICAL = 0.8     # N·s/m

# Délai de réponse moteur (filtre passe-bas).
# Produit des oscillations visibles avec PID mal réglé (Kd=0).
MOTOR_TAU = 0.07        # secondes

MIN_MOTOR = 10.0        # % — en dessous le moteur ne tourne pas


# ─────────────────────────────────────────────────────────────────
class DronePhysics:
    """
    Modèle physique 3 axes : roll, pitch, altitude.

    Les throttles moteurs passent par un filtre passe-bas (MOTOR_TAU)
    qui modélise le délai ESC/moteur. Ce délai rend le PID oscillant
    (Kd=0) visible à l'écran.

    apply_perturbation() injecte directement un décalage d'angle
    """

    def __init__(self):
        self.roll       = 0.0
        self.pitch      = 0.0
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0
        self.altitude   = 0.0
        self.vz         = 0.0
        self.on_ground  = True

        # États filtrés des moteurs
        self._m1 = 0.0
        self._m2 = 0.0
        self._m3 = 0.0
        self._m4 = 0.0

    # ─────────────────────────────────────────────────────────────
    def _thrust(self, pct: float) -> float:
        """Throttle % → force (N) pour UN moteur."""
        if pct < MIN_MOTOR:
            return 0.0
        return (min(pct, 100.0) / 100.0) * THRUST_PER_MOTOR

    # ─────────────────────────────────────────────────────────────
    def step(self, m1: float, m2: float, m3: float, m4: float, dt: float):
        """
        Intègre la physique sur dt secondes.

        m1..m4 : consignes throttle (%) envoyées par le FC.
        Elles passent par un filtre avant conversion en forces.

        Config X (vue dessus) :
          M2(CCW) --- M1(CW)
             |    X    |
          M3(CW)  --- M4(CCW)

        Couples :
          Roll  = (M2+M3 - M1-M4) * ARM   → gauche positif
          Pitch = (M1+M2 - M3-M4) * ARM   → avant positif
        """
        # Filtre passe-bas moteurs
        a = dt / (dt + MOTOR_TAU)
        self._m1 += a * (m1 - self._m1)
        self._m2 += a * (m2 - self._m2)
        self._m3 += a * (m3 - self._m3)
        self._m4 += a * (m4 - self._m4)

        f1 = self._thrust(self._m1)
        f2 = self._thrust(self._m2)
        f3 = self._thrust(self._m3)
        f4 = self._thrust(self._m4)
        total_thrust = f1 + f2 + f3 + f4

        # Couples angulaires
        torque_roll  = (f2 + f3 - f1 - f4) * ARM_LENGTH_M
        torque_pitch = (f1 + f2 - f3 - f4) * ARM_LENGTH_M

        # Dynamique angulaire
        rr_rad = math.radians(self.roll_rate)
        pr_rad = math.radians(self.pitch_rate)
        alpha_roll  = (torque_roll  - DRAG_ANGULAR * rr_rad)  / INERTIA_ROLL
        alpha_pitch = (torque_pitch - DRAG_ANGULAR * pr_rad) / INERTIA_PITCH

        self.roll_rate  += math.degrees(alpha_roll)  * dt
        self.pitch_rate += math.degrees(alpha_pitch) * dt
        self.roll       += self.roll_rate  * dt
        self.pitch      += self.pitch_rate * dt

        # Clamp physique
        self.roll  = max(-180.0, min(180.0, self.roll))
        self.pitch = max(-180.0, min(180.0, self.pitch))

        # Dynamique verticale
        tilt = math.cos(math.radians(self.roll)) * math.cos(math.radians(self.pitch))
        tilt = max(0.0, tilt)
        fz_net = total_thrust * tilt - weight - DRAG_VERTICAL * self.vz
        self.vz       += (fz_net / MASS_KG) * dt
        self.altitude += self.vz * dt

        # Sol
        if self.altitude <= 0.0:
            self.altitude = 0.0
            self.vz = max(0.0, self.vz)
            self.on_ground = (total_thrust < weight * 0.95)
        else:
            self.on_ground = False

    # ─────────────────────────────────────────────────────────────
    def apply_perturbation(self, roll_deg: float = 0.0, pitch_deg: float = 0.0):
        """
        Simule un coup de vent en injectant directement un décalage d'angle.
        """
        self.roll  += roll_deg
        self.pitch += pitch_deg

    # ─────────────────────────────────────────────────────────────
    def get_state(self) -> dict:
        return {
            "roll": self.roll, "pitch": self.pitch,
            "roll_rate": self.roll_rate, "pitch_rate": self.pitch_rate,
            "altitude": self.altitude, "vz": self.vz,
            "on_ground": self.on_ground,
        }

    # ─────────────────────────────────────────────────────────────
    def reset(self):
        self.roll = self.pitch = 0.0
        self.roll_rate = self.pitch_rate = 0.0
        self.altitude = self.vz = 0.0
        self.on_ground = True
        self._m1 = self._m2 = self._m3 = self._m4 = 0.0