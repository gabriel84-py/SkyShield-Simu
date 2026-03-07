import time


class PID:
    """Contrôleur PID av anti-windup, clamp dérivée et protection dt"""

    def __init__(self, kp, ki, kd, output_limits=(-100, 100), integral_limit=50):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_out, self.max_out = output_limits
        self.integral_limit = integral_limit

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None  # None = pas encore initialisé → évite pic D au démarrage

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def compute(self, setpoint, measurement):
        """
        Calc correction PID

        Params :
        - setpoint    : cible (ex: roll = 0°)
        - measurement : mesure actuelle (ex: roll = -5°)

        Ret : correction à appliquer
        """
        now = time.ticks_us()

        # ── FIX 1 : premier appel → init propre, retourne 0 ─────
        # Sans ce fix : last_time = moment __init__ du PID, premier appel
        # arrive ~2s plus tard (armement ESC) → dt = 2s → derivative =
        # (error - 0) / 2 mais surtout last_error = 0 ≠ vraie erreur →
        # spike D énorme dès le 1er sample (observé : d_pitch=50 à t=0.015s)
        if self.last_time is None:
            self.last_time = now
            self.last_error = setpoint - measurement
            return 0.0

        dt = time.ticks_diff(now, self.last_time) / 1_000_000  # secondes
        self.last_time = now

        # ── FIX 2 : dt aberrant → valeur nominale ────────────────
        # > 0.5s = gap boucle (time_pulse_us bloquant, freeze...)
        # Sans fix : gap 150ms avec angle changé → derivative explose
        if dt <= 0 or dt > 0.5:
            dt = 0.01  # dt nominal 100 Hz

        # ── erreur ───────────────────────────────────────────────
        error = setpoint - measurement

        # ── terme P ───────────────────────────────────────────────
        self.p_term = self.kp * error

        # ── terme I avec anti-windup ──────────────────────────────
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        self.i_term = self.ki * self.integral

        # ── FIX 3 : clamp dérivée brute avant × Kd ───────────────
        # Sans fix : gap boucle + grand angle = dérivée 200-1000
        # → corrections énormes sur 1 seul sample (observé : d_pitch=1001)
        derivative = (error - self.last_error) / dt
        derivative = max(-500.0, min(500.0, derivative))
        self.d_term = self.kd * derivative

        # ── somme et limite sortie ────────────────────────────────
        output = self.p_term + self.i_term + self.d_term
        output = max(self.min_out, min(self.max_out, output))

        self.last_error = error
        return output

    def reset(self):
        """Reset états internes — appelé à l'armement et à la coupure gaz"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None  # réinitialisé proprement au prochain compute()
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def set_gains(self, kp=None, ki=None, kd=None):
        """Change gains en direct (tuning live)"""
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd

    def get_terms(self):
        """Retourne termes P, I, D séparés (debug/logging)"""
        return self.p_term, self.i_term, self.d_term