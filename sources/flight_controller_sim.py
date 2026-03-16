"""
flight_controller_sim.py — Contrôleur de vol simulé

Miroir de flight_controller.py du vrai drone, SAUF :
  - MPU6050 remplacé par DronePhysics
  - ESC remplacées par DronePhysics
  - arm() / disarm() sans délai matériel
  - PID, mixage et seuils identiques
"""

from pid import PID
from physics import DronePhysics


class FlightControllerSim:

    def __init__(self):
        self.physics = DronePhysics()

        self.pid_roll = PID(
            kp=0.8, ki=0.00, kd=0.25,
            output_limits=(-25, 25), integral_limit=10
        )
        self.pid_pitch = PID(
            kp=0.8, ki=0.00, kd=0.25,
            output_limits=(-25, 25), integral_limit=10
        )

        self.target_roll  = 0.0
        self.target_pitch = 0.0
        self.base_throttle = 0.0
        self.max_throttle  = 30.0

        self.MIN_MOTOR = 10.0
        self.DEAD_ON   = 14.0
        self.DEAD_OFF  = 11.5
        self._throttle_active = False
        self._prev_throttle   = 0.0

        # Seuil safety à 60° — les oscillations du scénario PID culminent à ~28°
        self.max_angle      = 60.0
        self.armed          = False
        self.emergency_stop = False

        # Warmup : on ignore le check_safety pendant les 80 premiers steps (0.8s)
        # pour éviter un faux positif au démarrage
        self._warmup_steps = 0
        self._WARMUP_GRACE = 80

        self.m1 = self.m2 = self.m3 = self.m4 = 0.0
        self.corr_roll = self.corr_pitch = 0.0

    # ─────────────────────────────────────────────────────────────
    def arm(self):
        self.physics.reset()
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self._prev_throttle   = 0.0
        self._throttle_active = False
        self.m1 = self.m2 = self.m3 = self.m4 = 0.0
        self.corr_roll = self.corr_pitch = 0.0
        self.armed          = True
        self.emergency_stop = False
        self._warmup_steps  = 0

    def disarm(self):
        self.m1 = self.m2 = self.m3 = self.m4 = 0.0
        self.base_throttle = 0.0
        self.armed = False

    def emergency(self):
        self.emergency_stop = True
        self.disarm()

    # ─────────────────────────────────────────────────────────────
    def set_throttle(self, throttle_raw: float):
        if not self._throttle_active:
            if throttle_raw >= self.DEAD_ON:
                self._throttle_active = True
        else:
            if throttle_raw < self.DEAD_OFF:
                self._throttle_active = False

        new_t = 0.0 if not self._throttle_active \
                else max(0.0, min(self.max_throttle, throttle_raw))

        if new_t > 0 and self._prev_throttle == 0:
            self.pid_roll.reset(); self.pid_pitch.reset()
        if new_t == 0 and self._prev_throttle > 0:
            self.pid_roll.reset(); self.pid_pitch.reset()

        self._prev_throttle = new_t
        self.base_throttle  = new_t

    def set_pid_gains(self, axis: str, kp=None, ki=None, kd=None):
        pid = self.pid_roll if axis == 'roll' else self.pid_pitch
        pid.set_gains(kp, ki, kd)

    # ─────────────────────────────────────────────────────────────
    def _snap(self, v: float) -> float:
        # Clamp à max_throttle (30%), PAS à 100% !
        # Sans ce clamp, throttle=22 + corr=25 → moteur à 47%,
        # soit ×2.35 la force attendue → couple parasite → pitch diverge.
        v = max(0.0, min(self.max_throttle, v))
        if v <= 0.0: return 0.0
        if v < self.MIN_MOTOR: return self.MIN_MOTOR
        return v

    def _mix(self, thr, cr, cp):
        if thr <= 0:
            return 0.0, 0.0, 0.0, 0.0
        # Config X — signe de cp POSITIF sur M1/M2 (avant) et NÉGATIF sur M3/M4 (arrière).
        # pitch > 0 → corr_pitch < 0 → réduit avant, augmente arrière → couple négatif → corrige.
        return (
            self._snap(thr - cr + cp),   # M1 front-right
            self._snap(thr + cr + cp),   # M2 front-left
            self._snap(thr + cr - cp),   # M3 rear-left
            self._snap(thr - cr - cp),   # M4 rear-right
        )

    # ─────────────────────────────────────────────────────────────
    def check_safety(self, roll: float, pitch: float) -> bool:
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
        roll  = self.physics.roll
        pitch = self.physics.pitch

        if not self.check_safety(roll, pitch):
            return roll, pitch, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        if not self.armed or self.emergency_stop:
            return roll, pitch, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        cr = self.pid_roll.compute(self.target_roll, roll)
        cp = self.pid_pitch.compute(self.target_pitch, pitch)
        thr = max(0.0, min(self.max_throttle, self.base_throttle))
        m1, m2, m3, m4 = self._mix(thr, cr, cp)

        self.physics.step(m1, m2, m3, m4, dt)

        self.m1, self.m2, self.m3, self.m4 = m1, m2, m3, m4
        self.corr_roll, self.corr_pitch = cr, cp
        return roll, pitch, m1, m2, m3, m4, cr, cp

    # ───────────────────────────────────────────────────────────── c'est plus lisible comme ça non ?
    def get_pid_terms(self, axis: str) -> tuple:
        pid = self.pid_roll if axis == 'roll' else self.pid_pitch
        return pid.get_terms()

    @property # décorateur qui permet de transformer une méthode en attribut accessible comme une variable
    def altitude(self): return self.physics.altitude

    @property
    def vz(self): return self.physics.vz