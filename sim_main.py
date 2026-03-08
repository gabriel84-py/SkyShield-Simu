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
SIM_HZ      = 100
SIM_DT      = 1.0 / SIM_HZ
TARGET_FPS  = 60

UI_W        = 280

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
        self._t     = 0.0
        self._perturb_done = False

    def start(self, idx: int):
        self.active = idx
        self._t     = 0.0
        self._perturb_done = False
        self.fc.arm()
        self.fc.physics.reset()

        if idx == 0:
            # ── Stabilisation ─────────────────────────────────────
            # Hover à ~18%, throttle à 20% → légère montée puis vol stable
            self._set_gains(kp=0.8, ki=0.0, kd=0.25)
            self.fc.set_throttle(20.0)
            self.panel.sl_throttle.value = 20.0

        elif idx == 1:
            # ── Perturbation Roll ──────────────────────────────────
            # Gains réactifs, throttle stable au-dessus du hover
            self._set_gains(kp=1.0, ki=0.0, kd=0.35)
            self.fc.set_throttle(21.0)
            self.panel.sl_throttle.value = 21.0

        elif idx == 2:
            # ── PID oscillant ──────────────────────────────────────
            # Kp=2.5, Kd=0 : avec le délai moteur (MOTOR_TAU=0.07s),
            # ce réglage produit des oscillations stables ~25-30°.
            # Bien en dessous du seuil emergency (75°).
            self._set_gains(kp=2.5, ki=0.0, kd=0.0)
            self.fc.set_throttle(20.0)
            self.panel.sl_throttle.value = 20.0
            self.panel.sl_roll_kp.value  = 2.5
            self.panel.sl_pitch_kp.value = 2.5
            self.panel.sl_roll_kd.value  = 0.0
            self.panel.sl_pitch_kd.value = 0.0
            # La perturbation est injectée dans tick() après le warm-up

        elif idx == 3:
            # ── Décollage progressif ───────────────────────────────
            # Rampe 0 → 22% en 4s (hover ~18%), puis maintien
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
        if not self.fc.armed:
            return
        self._t += dt

        if self.active == 1:
            # Perturbation roll toutes les 4s, amplitude 15°/s
            if abs(self._t % 4.0) < dt * 1.5:
                sign = 1 if int(self._t / 4.0) % 2 == 0 else -1
                self.fc.physics.apply_perturbation(roll_deg=sign * 15.0)

        elif self.active == 2:
            # Perturbation initiale après la fin du warm-up (>0.9s) — une seule fois
            if self._t > 0.9 and not self._perturb_done:
                self.fc.physics.apply_perturbation(roll_deg=12.0, pitch_deg=8.0)
                self._perturb_done = True

        elif self.active == 3:
            # Rampe 0 → 22% en 4s, puis maintien
            if self._t <= 4.0:
                thr = (self._t / 4.0) * 22.0
                self.fc.set_throttle(thr)
                self.panel.sl_throttle.value = thr
            else:
                self.fc.set_throttle(22.0)
                self.panel.sl_throttle.value = 22.0


# ─────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    pygame.display.set_caption("SkyShield Simulator — Trophées NSI 2026")

    screen  = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    clock   = pygame.time.Clock()

    fc      = FlightControllerSim()
    vis     = Visualizer(WINDOW_W - UI_W, WINDOW_H)
    panel   = ControlPanel(WINDOW_W, WINDOW_H)
    scenario= Scenario(fc, panel)

    current_scenario_name = ""
    logger: FlightLogger | None = None

    # Initialisés à 0 pour éviter tout UnboundLocalError
    m1 = m2 = m3 = m4 = 0.0
    cr = cp = 0.0

    last_sim_time = time.perf_counter()

    print("=" * 50)
    print("  SkyShield Simulator — Trophées NSI 2026")
    print("=" * 50)
    print("  ESPACE  : ARM / DISARM")
    print("  R       : Reset")
    print("  1-4     : Scénarios")
    print("  ↑ / ↓   : Throttle +/- 1%")
    print("  ECHAP   : Quitter")
    print("  Hover ≈ 18% throttle")
    print("=" * 50)

    running = True
    while running:

        now       = time.perf_counter()
        elapsed   = now - last_sim_time
        last_sim_time = now

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_SPACE:
                    if fc.armed:
                        fc.disarm()
                        if logger and logger.is_running:
                            logger.stop(); logger = None
                    else:
                        fc.arm()
                        logger = FlightLogger(current_scenario_name or "manuel")
                        logger.start()

                elif event.key == pygame.K_r:
                    if logger and logger.is_running:
                        logger.stop(); logger = None
                    fc.disarm()
                    fc.physics.reset()
                    vis.reset_history()
                    current_scenario_name = ""
                    scenario.active = -1
                    m1 = m2 = m3 = m4 = 0.0
                    cr = cp = 0.0

                elif event.key == pygame.K_UP:
                    panel.sl_throttle.value = min(30.0, panel.sl_throttle.value + 1.0)
                elif event.key == pygame.K_DOWN:
                    panel.sl_throttle.value = max(0.0,  panel.sl_throttle.value - 1.0)

                elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    idx = event.key - pygame.K_1
                    scenario.start(idx)
                    current_scenario_name = SCENARIOS[idx]
                    vis.reset_history()
                    m1 = m2 = m3 = m4 = 0.0
                    cr = cp = 0.0

        # ── actions UI ────────────────────────────────────────────
        actions = panel.handle_events(events)

        if "arm" in actions:
            fc.arm()
            logger = FlightLogger(current_scenario_name or "manuel")
            logger.start()
        if "disarm" in actions:
            if logger and logger.is_running:
                logger.stop(); logger = None
            fc.disarm()
        if "reset" in actions:
            if logger and logger.is_running:
                logger.stop(); logger = None
            fc.disarm(); fc.physics.reset(); vis.reset_history()
            current_scenario_name = ""; panel.sl_throttle.value = 0.0
            scenario.active = -1; m1 = m2 = m3 = m4 = 0.0; cr = cp = 0.0

        if "pid_changed" in actions:
            kp_r, ki_r, kd_r = panel.roll_gains
            kp_p, ki_p, kd_p = panel.pitch_gains
            fc.set_pid_gains('roll',  kp=kp_r, ki=ki_r, kd=kd_r)
            fc.set_pid_gains('pitch', kp=kp_p, ki=ki_p, kd=kd_p)

        if "throttle" in actions:
            fc.set_throttle(actions["throttle"])

        if "perturb_roll"  in actions:
            fc.physics.apply_perturbation(roll_deg=actions["perturb_roll"])
        if "perturb_pitch" in actions:
            fc.physics.apply_perturbation(pitch_deg=actions["perturb_pitch"])

        if "scenario" in actions:
            idx = actions["scenario"]
            scenario.start(idx)
            current_scenario_name = SCENARIOS[idx]
            vis.reset_history()
            m1 = m2 = m3 = m4 = 0.0; cr = cp = 0.0

        # ── throttle manuel (hors scénario) ───────────────────────
        if fc.armed and scenario.active < 0:
            fc.set_throttle(panel.sl_throttle.value)

        # ── steps de simulation ────────────────────────────────────
        steps = max(1, int(elapsed * SIM_HZ))
        steps = min(steps, 10)

        for _ in range(steps):
            scenario.tick(SIM_DT)
            result = fc.update(dt=SIM_DT)
            roll, pitch, m1, m2, m3, m4, cr, cp = result

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

        if logger and logger.is_running and fc.armed:
            logger.log(state, pid_terms)

        vis.draw(state, pid_terms, current_scenario_name)
        screen.blit(vis.screen, (0, 0))
        panel.draw(screen)
        pygame.display.flip()
        clock.tick(TARGET_FPS)

    if logger and logger.is_running:
        logger.stop()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()