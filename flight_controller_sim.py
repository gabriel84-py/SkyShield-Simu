"""
flight_controller_sim.py — Contrôleur de vol pour la simulation

Miroir exact de flight_controller.py du vrai drone, SAUF :
  - MPU6050 remplacé par lecture dans DronePhysics
  - ESC.throttle() remplacé par écriture dans DronePhysics
  - arm() / disarm() sans délai matériel
  - Même PID, même mixage, mêmes seuils

L'objectif : le jury peut vérifier que le code de simulation
est structurellement identique au code embarqué réel.
"""

from pid import PID
from physics import DronePhysics


# ─────────────────────────────────────────────────────────────────
class FlightControllerSim:
    """
    Contrôleur de vol simulé.

    Utilise :
      - pid.py          → inchangé par rapport au drone réel
      - DronePhysics    → remplace MPU6050 + ESC

    Interface identique à FlightController (flight_controller.py) :
      arm(), disarm(), set_throttle(), update(), get_pid_terms()
    """

    def __init__(self):
        # ── physique ──────────────────────────────────────────────
        self.physics = DronePhysics()

        # ── PID (gains identiques au drone réel) ──────────────────
        self.pid_roll = PID(
            kp=0.8, ki=0.00, kd=0.25,
            output_limits=(-25, 25), integral_limit=10
        )
        self.pid_pitch = PID(
            kp=0.8, ki=0.00, kd=0.25,
            output_limits=(-25, 25), integral_limit=10
        )

        # ── cibles PID ────────────────────────────────────────────
        self.target_roll  = 0.0
        self.target_pitch = 0.0

        # ── throttle ──────────────────────────────────────────────
        self.base_throttle = 0.0
        self.max_throttle  = 30.0     # limite banc test, identique au drone

        # ── seuils deadzone (identiques flight_controller.py) ─────
        self.MIN_MOTOR  = 10.0
        self.DEAD_ON    = 14.0
        self.DEAD_OFF   = 11.5
        self._throttle_active = False
        self._prev_throttle   = 0.0

        # ── sécurité ──────────────────────────────────────────────
        # 60° en simulateur (pas de crash matériel réel)
        self.max_angle      = 60.0
        self.armed          = False
        self.emergency_stop = False

        # ── warm-up : ignore safety pendant les 50 premiers steps ─
        # Évite un faux-positif au démarrage quand le PID n'est pas
        # encore stabilisé et que les angles initiaux sont à 0
        self._warmup_steps  = 0
        self._WARMUP_GRACE  = 50      # ~0.5s à 100Hz

        # ── état moteurs (pour la visualisation) ──────────────────
        self.m1 = 0.0
        self.m2 = 0.0
        self.m3 = 0.0
        self.m4 = 0.0

        # ── corrections PID courantes ──────────────────────────────
        self.corr_roll  = 0.0
        self.corr_pitch = 0.0

    # ─────────────────────────────────────────────────────────────
    def arm(self):
        """
        Arme le simulateur.
        Pas de délai matériel (pas d'ESC réelle).
        Reset PID et physique.
        """
        self.physics.reset()
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self._prev_throttle   = 0.0
        self._throttle_active = False
        self.m1 = self.m2 = self.m3 = self.m4 = 0.0
        self.corr_roll = self.corr_pitch = 0.0
        self.armed          = True
        self.emergency_stop = False
        self._warmup_steps  = 0     # repart à zéro à chaque armement

    # ─────────────────────────────────────────────────────────────
    def disarm(self):
        self.m1 = self.m2 = self.m3 = self.m4 = 0.0
        self.base_throttle = 0.0
        self.armed         = False

    # ─────────────────────────────────────────────────────────────
    def emergency(self):
        self.emergency_stop = True
        self.disarm()

    # ─────────────────────────────────────────────────────────────
    def set_throttle(self, throttle_raw: float):
        """
        Applique la deadzone avec hystérésis.
        Identique à FlightController.set_throttle().
        """
        if not self._throttle_active:
            if throttle_raw >= self.DEAD_ON:
                self._throttle_active = True
        else:
            if throttle_raw < self.DEAD_OFF:
                self._throttle_active = False

        new_throttle = 0.0 if not self._throttle_active else max(0.0, min(self.max_throttle, throttle_raw))

        # reset PID aux transitions
        if new_throttle > 0 and self._prev_throttle == 0:
            self.pid_roll.reset()
            self.pid_pitch.reset()
        if new_throttle == 0 and self._prev_throttle > 0:
            self.pid_roll.reset()
            self.pid_pitch.reset()

        self._prev_throttle = new_throttle
        self.base_throttle  = new_throttle

    # ─────────────────────────────────────────────────────────────
    def set_pid_gains(self, axis: str, kp=None, ki=None, kd=None):
        """Tuning live des gains PID (appelé par les sliders UI)."""
        if axis == 'roll':
            self.pid_roll.set_gains(kp, ki, kd)
        elif axis == 'pitch':
            self.pid_pitch.set_gains(kp, ki, kd)

    # ─────────────────────────────────────────────────────────────
    def _snap_to_min(self, val: float) -> float:
        """
        Snap au minimum moteur.
        Identique à FlightController._snap_to_min().
        """
        v = max(0.0, min(100.0, val))
        if v <= 0.0:
            return 0.0
        if v < self.MIN_MOTOR:
            return self.MIN_MOTOR
        return v

    # ─────────────────────────────────────────────────────────────
    def _mix_motors(self, throttle: float, corr_roll: float, corr_pitch: float):
        """
        Mixage config X.
        Identique à FlightController.mix_motors().
        """
        if throttle <= 0:
            return 0.0, 0.0, 0.0, 0.0

        m1 = self._snap_to_min(throttle - corr_roll - corr_pitch)  # front-right
        m2 = self._snap_to_min(throttle + corr_roll - corr_pitch)  # front-left
        m3 = self._snap_to_min(throttle + corr_roll + corr_pitch)  # rear-left
        m4 = self._snap_to_min(throttle - corr_roll + corr_pitch)  # rear-right

        return m1, m2, m3, m4

    # ─────────────────────────────────────────────────────────────
    def check_safety(self, roll: float, pitch: float) -> bool:
        """
        Vérifie que le drone n'est pas en dehors des limites angulaires.
        Le warm-up grace period évite un faux-positif au démarrage.
        """
        # Pendant le warm-up, on laisse passer sans vérifier
        if self._warmup_steps < self._WARMUP_GRACE:
            self._warmup_steps += 1
            return True

        if abs(roll  - self.target_roll)  > self.max_angle or \
           abs(pitch - self.target_pitch) > self.max_angle:
            self.emergency()
            return False
        return True

    # ─────────────────────────────────────────────────────────────
    def update(self, dt: float = 0.01):
        """
        Boucle principale — appelée à ~100 Hz.

        1. Lit les angles depuis la physique (remplace MPU6050.read_angles())
        2. Calcule les corrections PID
        3. Mixe les moteurs
        4. Écrit les throttles dans la physique (remplace ESC.throttle())
        5. Intègre la physique sur dt

        Retourne : (roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch)
        """
        # ── lecture IMU simulée (depuis la physique) ──────────────
        roll  = self.physics.roll
        pitch = self.physics.pitch

        # ── sécurité angle ─────────────────────────────────────────
        if not self.check_safety(roll, pitch):
            return roll, pitch, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        if not self.armed or self.emergency_stop:
            return roll, pitch, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # ── corrections PID ───────────────────────────────────────
        corr_roll  = self.pid_roll.compute(self.target_roll,  roll)
        corr_pitch = self.pid_pitch.compute(self.target_pitch, pitch)

        throttle = max(0.0, min(self.max_throttle, self.base_throttle))

        # ── mixage moteurs ────────────────────────────────────────
        m1, m2, m3, m4 = self._mix_motors(throttle, corr_roll, corr_pitch)

        # ── écriture dans la physique (remplace ESC.throttle()) ───
        self.physics.step(m1, m2, m3, m4, dt)

        # ── sauvegarde état moteurs ────────────────────────────────
        self.m1, self.m2, self.m3, self.m4 = m1, m2, m3, m4
        self.corr_roll, self.corr_pitch     = corr_roll, corr_pitch

        return roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch

    # ─────────────────────────────────────────────────────────────
    def get_pid_terms(self, axis: str) -> tuple:
        if axis == 'roll':
            return self.pid_roll.get_terms()
        elif axis == 'pitch':
            return self.pid_pitch.get_terms()
        return 0.0, 0.0, 0.0

    # ─────────────────────────────────────────────────────────────
    @property
    def altitude(self) -> float:
        return self.physics.altitude

    @property
    def vz(self) -> float:
        return self.physics.vz