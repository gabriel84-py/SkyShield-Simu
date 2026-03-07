"""
pid.py — Contrôleur PID (adapté simulation PC)

Identique au pid.py du drone réel, SAUF :
  - time.ticks_us()   → time.perf_counter() * 1_000_000
  - time.ticks_diff() → soustraction directe

La logique PID (anti-windup, clamp dérivée, protection dt) est INCHANGÉE.
C'est le même algorithme qui tourne sur le Pico RP2040.
"""

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
        self.integral   = 0.0
        self.last_time  = None   # None = pas encore initialisé → évite pic D

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    # ── compatibilité MicroPython → Python standard ───────────────
    @staticmethod
    def _now_us() -> float:
        """Temps courant en microsecondes (float). Remplace time.ticks_us()."""
        return time.perf_counter() * 1_000_000

    # ─────────────────────────────────────────────────────────────
    def compute(self, setpoint: float, measurement: float) -> float:
        """
        Calcule la correction PID.

        Params :
          setpoint    : cible   (ex : roll = 0°)
          measurement : mesure  (ex : roll = -5°)

        Retourne : correction à appliquer (%)
        """
        now = self._now_us()

        # ── FIX 1 : premier appel → init propre, retourne 0 ──────
        if self.last_time is None:
            self.last_time  = now
            self.last_error = setpoint - measurement
            return 0.0

        dt = (now - self.last_time) / 1_000_000   # secondes
        self.last_time = now

        # ── FIX 2 : dt aberrant → valeur nominale ─────────────────
        if dt <= 0 or dt > 0.5:
            dt = 0.01   # 100 Hz nominal

        # ── erreur ────────────────────────────────────────────────
        error = setpoint - measurement

        # ── terme P ───────────────────────────────────────────────
        self.p_term = self.kp * error

        # ── terme I avec anti-windup ───────────────────────────────
        self.integral += error * dt
        self.integral  = max(-self.integral_limit, min(self.integral_limit, self.integral))
        self.i_term    = self.ki * self.integral

        # ── FIX 3 : clamp dérivée brute avant × Kd ────────────────
        derivative  = (error - self.last_error) / dt
        derivative  = max(-500.0, min(500.0, derivative))
        self.d_term = self.kd * derivative

        # ── sortie clampée ─────────────────────────────────────────
        output = self.p_term + self.i_term + self.d_term
        output = max(self.min_out, min(self.max_out, output))

        self.last_error = error
        return output

    # ─────────────────────────────────────────────────────────────
    def reset(self):
        """Reset états — appelé à chaque armement ou coupure gaz."""
        self.integral   = 0.0
        self.last_error = 0.0
        self.last_time  = None
        self.p_term     = 0.0
        self.i_term     = 0.0
        self.d_term     = 0.0

    def set_gains(self, kp=None, ki=None, kd=None):
        """Change les gains en direct (tuning live)."""
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd

    def get_terms(self) -> tuple:
        """Retourne (P, I, D) séparés pour le debug et le logging."""
        return self.p_term, self.i_term, self.d_term