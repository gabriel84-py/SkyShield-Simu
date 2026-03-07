"""
sim_main.py — SkyShield Simulator
Point d'entrée principal

Boucle principale :
  1. Événements Pygame (sliders, boutons, clavier)
  2. Mise à jour FC simulé (PID + physique)
  3. Rendu Pygame (visualizer + UI)

Scénarios disponibles (touches clavier ou boutons UI) :
  1 — Stabilisation à plat (throttle 20%)
  2 — Perturbation Roll automatique
  3 — PID oscillant (gains trop élevés)
  4 — Décollage progressif

Raccourcis clavier :
  ESPACE  — ARM / DISARM
  R       — Reset
  1-4     — Sélection scénario
  ↑/↓     — Throttle +/- 1%
  ECHAP   — Quitter
"""

import sys
import time
import pygame

from flight_controller_sim import FlightControllerSim
from visualizer import Visualizer
from ui import ControlPanel
from logger import FlightLogger

# ─────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────
WINDOW_W    = 1200
WINDOW_H    = 750
SIM_HZ      = 100          # fréquence de simulation (Hz)
SIM_DT      = 1.0 / SIM_HZ
TARGET_FPS  = 60           # fréquence d'affichage (Hz)

UI_W        = 280          # largeur panneau UI droite

# ─────────────────────────────────────────────────────────────────
# Scénarios
# ─────────────────────────────────────────────────────────────────
SCENARIOS = {
    0: "Stabilisation",
    1: "Perturbation Roll",
    2: "PID oscillant",
    3: "Décollage progressif",
}


# ─────────────────────────────────────────────────────────────────
class Scenario:
    """Pilote automatique pour les scénarios de démo."""

    def __init__(self, fc: FlightControllerSim, panel: ControlPanel):
        self.fc     = fc
        self.panel  = panel
        self.active = -1
        self._t     = 0.0   # temps interne scénario

    def start(self, idx: int):
        """Lance un scénario — reset FC et configure gains si besoin."""
        self.active = idx
        self._t     = 0.0
        self.fc.arm()
        self.fc.physics.reset()

        if idx == 0:   # stabilisation — gains normaux
            self._set_gains(kp=0.8, ki=0.0, kd=0.25)
            self.fc.set_throttle(20.0)
            self.panel.sl_throttle.value = 20.0

        elif idx == 1: # perturbation roll
            self._set_gains(kp=0.8, ki=0.0, kd=0.25)
            self.fc.set_throttle(22.0)
            self.panel.sl_throttle.value = 22.0

        elif idx == 2: # PID oscillant — Kp trop élevé
            self._set_gains(kp=4.5, ki=0.0, kd=0.0)
            self.fc.set_throttle(20.0)
            self.panel.sl_throttle.value = 20.0
            # màj sliders pour que l'utilisateur voie les gains
            self.panel.sl_roll_kp.value  = 4.5
            self.panel.sl_pitch_kp.value = 4.5
            self.panel.sl_roll_kd.value  = 0.0
            self.panel.sl_pitch_kd.value = 0.0

        elif idx == 3: # décollage progressif
            self._set_gains(kp=0.8, ki=0.0, kd=0.25)
            self.fc.set_throttle(0.0)
            self.panel.sl_throttle.value = 0.0

    def _set_gains(self, kp, ki, kd):
        self.fc.set_pid_gains('roll',  kp=kp, ki=ki, kd=kd)
        self.fc.set_pid_gains('pitch', kp=kp, ki=ki, kd=kd)
        self.panel.sl_roll_kp.value  = kp
        self.panel.sl_roll_ki.value  = ki
        self.panel.sl_roll_kd.value  = kd
        self.panel.sl_pitch_kp.value = kp
        self.panel.sl_pitch_ki.value = ki
        self.panel.sl_pitch_kd.value = kd

    def tick(self, dt: float):
        """Appelé à chaque step de simulation — injecte des événements automatiques."""
        if not self.fc.armed:
            return

        self._t += dt

        if self.active == 1:   # perturbation roll toutes les 3s
            if abs(self._t % 3.0) < dt * 1.5:
                sign = 1 if int(self._t / 3.0) % 2 == 0 else -1
                self.fc.physics.apply_perturbation(roll_deg=sign * 40.0)

        elif self.active == 2: # perturbation initiale pour déclencher oscillations
            if self._t < dt * 2:
                self.fc.physics.apply_perturbation(roll_deg=15.0, pitch_deg=10.0)

        elif self.active == 3: # rampe throttle : 0 → 25% en 5s
            if self._t <= 5.0:
                thr = (self._t / 5.0) * 25.0
                self.fc.set_throttle(thr)
                self.panel.sl_throttle.value = thr


# ─────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    pygame.display.set_caption("SkyShield Simulator — Trophées NSI 2026")

    screen  = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    clock   = pygame.time.Clock()

    # ── composants ────────────────────────────────────────────────
    fc      = FlightControllerSim()
    vis     = Visualizer(WINDOW_W - UI_W, WINDOW_H)
    panel   = ControlPanel(WINDOW_W, WINDOW_H)
    scenario= Scenario(fc, panel)

    # état global
    current_scenario_name = ""
    sim_steps_per_frame   = max(1, SIM_HZ // TARGET_FPS)
    logger: FlightLogger | None = None   # logger actif (None = pas de vol en cours)

    # accumulateur de temps pour la simulation
    last_sim_time = time.perf_counter()

    print("=" * 50)
    print("  SkyShield Simulator — Trophées NSI 2026")
    print("=" * 50)
    print("  ESPACE  : ARM / DISARM")
    print("  R       : Reset")
    print("  1-4     : Scénarios")
    print("  ↑ / ↓   : Throttle +/- 1%")
    print("  ECHAP   : Quitter")
    print("=" * 50)

    running = True
    while running:

        # ── gestion du temps simulation ───────────────────────────
        now       = time.perf_counter()
        elapsed   = now - last_sim_time
        last_sim_time = now

        # ── collecte événements ────────────────────────────────────
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False

            # ── clavier ───────────────────────────────────────────
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    if fc.armed:
                        fc.disarm()
                        if logger and logger.is_running:
                            logger.stop()
                            logger = None
                    else:
                        fc.arm()
                        logger = FlightLogger(current_scenario_name or "manuel")
                        logger.start()
                elif event.key == pygame.K_r:
                    if logger and logger.is_running:
                        logger.stop()
                        logger = None
                    fc.disarm()
                    fc.physics.reset()
                    vis.reset_history()
                    current_scenario_name = ""
                elif event.key == pygame.K_UP:
                    panel.sl_throttle.value = min(30.0, panel.sl_throttle.value + 1.0)
                elif event.key == pygame.K_DOWN:
                    panel.sl_throttle.value = max(0.0,  panel.sl_throttle.value - 1.0)
                elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    idx = event.key - pygame.K_1
                    scenario.start(idx)
                    current_scenario_name = SCENARIOS[idx]
                    vis.reset_history()

        # ── traitement actions UI ──────────────────────────────────
        actions = panel.handle_events(events)

        if "arm" in actions:
            fc.arm()
            logger = FlightLogger(current_scenario_name or "manuel")
            logger.start()
        if "disarm" in actions:
            if logger and logger.is_running:
                logger.stop()
                logger = None
            fc.disarm()
        if "reset" in actions:
            if logger and logger.is_running:
                logger.stop()
                logger = None
            fc.disarm()
            fc.physics.reset()
            vis.reset_history()
            current_scenario_name = ""
            panel.sl_throttle.value = 0.0
            scenario.active = -1

        if "pid_changed" in actions:
            kp_r, ki_r, kd_r = panel.roll_gains
            kp_p, ki_p, kd_p = panel.pitch_gains
            fc.set_pid_gains('roll',  kp=kp_r, ki=ki_r, kd=kd_r)
            fc.set_pid_gains('pitch', kp=kp_p, ki=ki_p, kd=kd_p)

        if "throttle" in actions:
            fc.set_throttle(actions["throttle"])

        if "perturb_roll" in actions:
            fc.physics.apply_perturbation(roll_deg=actions["perturb_roll"])
        if "perturb_pitch" in actions:
            fc.physics.apply_perturbation(pitch_deg=actions["perturb_pitch"])

        if "scenario" in actions:
            idx = actions["scenario"]
            scenario.start(idx)
            current_scenario_name = SCENARIOS[idx]
            vis.reset_history()

        # ── throttle continu depuis slider ─────────────────────────
        if fc.armed and scenario.active < 0:
            fc.set_throttle(panel.sl_throttle.value)

        # ── steps de simulation ────────────────────────────────────
        # On calcule combien de steps sim faire pour rester à SIM_HZ
        steps = max(1, int(elapsed * SIM_HZ))
        steps = min(steps, 10)   # cap : évite spiral of death si lag

        for _ in range(steps):
            scenario.tick(SIM_DT)
            roll, pitch, m1, m2, m3, m4, cr, cp = fc.update(dt=SIM_DT)

        # ── état pour le rendu ─────────────────────────────────────
        state = {
            "roll":       fc.physics.roll,
            "pitch":      fc.physics.pitch,
            "altitude":   fc.physics.altitude,
            "vz":         fc.physics.vz,
            "m1": m1, "m2": m2, "m3": m3, "m4": m4,
            "throttle":   fc.base_throttle,
            "armed":      fc.armed,
            "emergency":  fc.emergency_stop,
            "corr_roll":  cr,
            "corr_pitch": cp,
        }

        pid_terms = {
            "roll":  fc.get_pid_terms('roll'),
            "pitch": fc.get_pid_terms('pitch'),
        }

        vis.push_state(state)

        # ── logging ───────────────────────────────────────────────
        if logger and logger.is_running and fc.armed:
            logger.log(state, pid_terms)

        # ── rendu ─────────────────────────────────────────────────
        vis.draw(state, pid_terms, current_scenario_name)

        # ── copie sur l'écran principal ────────────────────────────
        screen.blit(vis.screen, (0, 0))
        panel.draw(screen)
        pygame.display.flip()

        clock.tick(TARGET_FPS)

    if logger and logger.is_running:
        logger.stop()
    pygame.quit()
    sys.exit(0)


# ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()