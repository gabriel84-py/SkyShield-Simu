"""
-Sido
logger.py — Logging des vols simulés SkyShield

Enregistre chaque vol dans data/vol_YYYYMMDD_HHMMSS.csv
Colonnes : timestamp, roll, pitch, altitude, vz,
           m1, m2, m3, m4, throttle,
           p_roll, i_roll, d_roll,
           p_pitch, i_pitch, d_pitch,
           corr_roll, corr_pitch, scenario
"""

import csv
import os
from datetime import datetime


DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


class FlightLogger:
    """
    Logger CSV pour un vol simulé.

    Usage :
        logger = FlightLogger(scenario_name="Stabilisation")
        logger.start()
        # dans la boucle :
        logger.log(state, pid_terms)
        # à la fin :
        logger.stop()
        print(logger.filepath)
    """

    COLUMNS = [
        "t_ms",
        "roll", "pitch",
        "altitude", "vz",
        "m1", "m2", "m3", "m4",
        "throttle",
        "p_roll", "i_roll", "d_roll",
        "p_pitch", "i_pitch", "d_pitch",
        "corr_roll", "corr_pitch",
        "scenario",
    ]

    def __init__(self, scenario_name: str = ""):
        os.makedirs(DATA_DIR, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # slug du scénario dans le nom de fichier
        slug = scenario_name.lower().replace(" ", "_")[:20] if scenario_name else "vol"
        filename = f"vol_{ts}_{slug}.csv" if slug else f"vol_{ts}.csv"

        self.filepath = os.path.join(DATA_DIR, filename)
        self.scenario_name = scenario_name

        self._file    = None
        self._writer  = None
        self._t0_ms   = None
        self._running = False
        self._rows    = 0

    # ─────────────────────────────────────────────────────────────
    def start(self):
        """Ouvre le fichier CSV et écrit l'en-tête."""
        if self._running:
            return
        self._file   = open(self.filepath, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.COLUMNS)
        self._writer.writeheader()
        self._t0_ms  = _now_ms()
        self._running = True
        self._rows    = 0
        print(f"[Logger] Démarré → {self.filepath}")

    # ─────────────────────────────────────────────────────────────
    def log(self, state: dict, pid_terms: dict):
        """
        Enregistre une ligne.

        state     : dict avec roll, pitch, altitude, vz, m1..m4,
                    throttle, corr_roll, corr_pitch
        pid_terms : dict avec "roll": (p, i, d) et "pitch": (p, i, d)
        """
        if not self._running:
            return

        t_ms = _now_ms() - self._t0_ms

        pr, ir, dr = pid_terms.get("roll",  (0.0, 0.0, 0.0))
        pp, ip, dp = pid_terms.get("pitch", (0.0, 0.0, 0.0))

        row = {
            "t_ms":      round(t_ms, 1),
            "roll":      round(state.get("roll",      0.0), 4),
            "pitch":     round(state.get("pitch",     0.0), 4),
            "altitude":  round(state.get("altitude",  0.0), 4),
            "vz":        round(state.get("vz",        0.0), 4),
            "m1":        round(state.get("m1",        0.0), 2),
            "m2":        round(state.get("m2",        0.0), 2),
            "m3":        round(state.get("m3",        0.0), 2),
            "m4":        round(state.get("m4",        0.0), 2),
            "throttle":  round(state.get("throttle",  0.0), 2),
            "p_roll":    round(pr, 4),
            "i_roll":    round(ir, 4),
            "d_roll":    round(dr, 4),
            "p_pitch":   round(pp, 4),
            "i_pitch":   round(ip, 4),
            "d_pitch":   round(dp, 4),
            "corr_roll":  round(state.get("corr_roll",  0.0), 4),
            "corr_pitch": round(state.get("corr_pitch", 0.0), 4),
            "scenario":   self.scenario_name,
        }
        self._writer.writerow(row)
        self._rows += 1

        # flush toutes les 100 lignes pour ne pas perdre les données en cas de crash
        if self._rows % 100 == 0:
            self._file.flush()

    # ─────────────────────────────────────────────────────────────
    def stop(self):
        """Ferme le fichier et affiche un résumé."""
        if not self._running:
            return
        self._file.flush()
        self._file.close()
        self._running = False
        duration_s = (_now_ms() - self._t0_ms) / 1000.0
        print(f"[Logger] Arrêté — {self._rows} lignes, {duration_s:.1f}s → {self.filepath}")

    # ─────────────────────────────────────────────────────────────
    @property
    def is_running(self) -> bool:
        return self._running

    @property # décorateur qui permet de transformer une méthode en attribut accessible comme une variable
    def row_count(self) -> int:
        return self._rows


# ─────────────────────────────────────────────────────────────────
def list_logs() -> list[str]:
    """Retourne la liste des fichiers de log triés du plus récent au plus ancien."""
    if not os.path.isdir(DATA_DIR):
        return []
    files = [f for f in os.listdir(DATA_DIR) if f.endswith(".csv")]
    files.sort(reverse=True)
    return [os.path.join(DATA_DIR, f) for f in files]


def _now_ms() -> float:
    """Timestamp en millisecondes."""
    import time
    return time.perf_counter() * 1000.0