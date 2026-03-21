"""
main.py — SkyShield Simulator — Point d'entrée principal

Scénarios :
  1 — Stabilisation (throttle 22%, hover=20%)
  2 — Perturbation Roll (décalage d'angle ±15° toutes les 4s)
  3 — PID oscillant (Kp=2.5, Kd=0 → oscillations ~28°)
  4 — Décollage progressif (rampe 0→24% en 5s)

Raccourcis :
  ESPACE  : ARM / DISARM     R : Reset
  1-4     : Scénarios        ↑/↓ : Throttle ±1%
  ECHAP   : Quitter
"""

import sys
import time
import pygame

from flight_controller_sim import FlightControllerSim
from visualizer import Visualizer
from ui import ControlPanel
from logger import FlightLogger

WINDOW_W   = 1200
WINDOW_H   = 750
SIM_HZ     = 100
SIM_DT     = 1.0 / SIM_HZ
TARGET_FPS = 60
UI_W       = 280

SCENARIOS = {
    0: "Stabilisation",
    1: "Perturbation Roll",
    2: "PID oscillant",
    3: "Décollage progressif",
}


# ─────────────────────────────────────────────────────────────────
class Scenario:

    def __init__(self, fc: FlightControllerSim, panel: ControlPanel):
        self.fc    = fc
        self.panel = panel
        self.active = -1
        self._t    = 0.0
        self._perturb_idx = 0  # compteur de perturbations

    # ─────────────────────────────────────────────────────────────
    def start(self, idx: int):
        self.active = idx
        self._t     = 0.0
        self._perturb_idx = 0
        self.fc.arm()

        if idx == 0:
            # Stabilisation : hover=20%, throttle=22% → monte doucement
            self._gains(kp=0.8, ki=0.0, kd=0.25)
            self._thr(22.0)

        elif idx == 1:
            # Perturbation Roll : bons gains, throttle stable
            # Perturbations = décalage d'angle direct ±15° toutes les 4s
            self._gains(kp=0.8, ki=0.0, kd=0.25)
            self._thr(22.0)

        elif idx == 2:
            # PID oscillant : Kp=2.5, Kd=0
            # Avec MOTOR_TAU=0.07s le système oscille à ~28° — visible, non fatal
            self._gains(kp=2.5, ki=0.0, kd=0.0)
            self._thr(29.0)
            self.panel.sl_roll_kp.value  = 2.5
            self.panel.sl_pitch_kp.value = 2.5
            self.panel.sl_roll_kd.value  = 0.0
            self.panel.sl_pitch_kd.value = 0.0

        elif idx == 3:
            # Décollage progressif : rampe 0→24% en 5s
            self._gains(kp=0.8, ki=0.0, kd=0.25)
            self._thr(0.0)

    # ─────────────────────────────────────────────────────────────
    def _gains(self, kp, ki, kd):
        self.fc.set_pid_gains('roll',  kp=kp, ki=ki, kd=kd)
        self.fc.set_pid_gains('pitch', kp=kp, ki=ki, kd=kd)
        self.panel.sl_roll_kp.value  = kp
        self.panel.sl_roll_ki.value  = ki
        self.panel.sl_roll_kd.value  = kd
        self.panel.sl_pitch_kp.value = kp
        self.panel.sl_pitch_ki.value = ki
        self.panel.sl_pitch_kd.value = kd

    def _thr(self, v: float):
        self.fc.set_throttle(v)
        self.panel.sl_throttle.value = v

    # ─────────────────────────────────────────────────────────────
    def tick(self, dt: float):
        if not self.fc.armed:
            return
        self._t += dt

        if self.active == 1:
            # Perturbation d'angle direct toutes les 4s
            # (plus visible qu'une injection de vitesse angulaire)
            interval = 4.0
            tick_idx = int(self._t / interval)
            frac = self._t - tick_idx * interval
            if frac < dt * 1.5 and tick_idx != self._perturb_idx:
                self._perturb_idx = tick_idx
                sign = 1 if tick_idx % 2 == 0 else -1
                self.fc.physics.apply_perturbation(roll_deg=sign * 15.0)

        elif self.active == 2:
            # Perturbation initiale après le warmup (3s) — une seule fois
            if self._t > 3 and self._perturb_idx == 0:
                self._perturb_idx = 1
                self.fc.physics.apply_perturbation(roll_deg=12.0, pitch_deg=8.0)

        elif self.active == 3:
            # Rampe 0→24% en 5s, puis maintien
            if self._t <= 5.0:
                thr = (self._t / 5.0) * 24.0
                self._thr(thr)
            elif self._t <= 5.0 + dt * 2:
                self._thr(24.0)


# ─────────────────────────────────────────────────────────────────
def main():
    pygame.init()

    screen   = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    clock    = pygame.time.Clock()
    pygame.display.set_caption("SkyShield Simulator — Trophées NSI 2026")

    fc       = FlightControllerSim()
    vis      = Visualizer(WINDOW_W - UI_W, WINDOW_H)
    panel    = ControlPanel(WINDOW_W, WINDOW_H)
    scenario = Scenario(fc, panel)

    current_scene = ""
    logger: FlightLogger | None = None
    m1 = m2 = m3 = m4 = cr = cp = 0.0

    last_t = time.perf_counter()

    print("=" * 52)
    print("  SkyShield Simulator — Trophées NSI 2026")
    print("  hover ≈ 20% | décolle à ~22% | max 30%")
    print("=" * 52)
    print("  ESPACE : ARM/DISARM    R : Reset")
    print("  1-4    : Scénarios     ↑↓ : Throttle ±1%")
    print("  ECHAP  : Quitter")
    print("=" * 52)

    running = True
    while running:
        now     = time.perf_counter()
        elapsed = now - last_t
        last_t  = now

        events = pygame.event.get()
        for ev in events:
            if ev.type == pygame.QUIT:
                running = False

            if ev.type == pygame.KEYDOWN:
                k = ev.key
                if k == pygame.K_ESCAPE:
                    running = False

                elif k == pygame.K_SPACE:
                    if fc.armed:
                        fc.disarm()
                        if logger: logger.stop(); logger = None
                    else:
                        fc.arm()
                        logger = FlightLogger(current_scene or "manuel")
                        logger.start()

                elif k == pygame.K_r:
                    if logger: logger.stop(); logger = None
                    fc.disarm(); fc.physics.reset(); vis.reset_history()
                    current_scene = ""; scenario.active = -1
                    m1 = m2 = m3 = m4 = cr = cp = 0.0

                elif k == pygame.K_UP:
                    panel.sl_throttle.value = min(30.0, panel.sl_throttle.value + 1.0)
                elif k == pygame.K_DOWN:
                    panel.sl_throttle.value = max(0.0,  panel.sl_throttle.value - 1.0)

                elif k in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    idx = k - pygame.K_1
                    if logger: logger.stop(); logger = None
                    scenario.start(idx)
                    current_scene = SCENARIOS[idx]
                    vis.reset_history()
                    m1 = m2 = m3 = m4 = cr = cp = 0.0
                    logger = FlightLogger(current_scene)
                    logger.start()

        # ── actions UI ────────────────────────────────────────────
        actions = panel.handle_events(events)

        if "arm" in actions:
            fc.arm()
            if logger: logger.stop()
            logger = FlightLogger(current_scene or "manuel"); logger.start()
        if "disarm" in actions:
            if logger: logger.stop(); logger = None
            fc.disarm()
        if "reset" in actions:
            if logger: logger.stop(); logger = None
            fc.disarm(); fc.physics.reset(); vis.reset_history()
            current_scene = ""; panel.sl_throttle.value = 0.0
            scenario.active = -1; m1=m2=m3=m4=cr=cp=0.0

        if "pid_changed" in actions:
            fc.set_pid_gains('roll',  *panel.roll_gains)
            fc.set_pid_gains('pitch', *panel.pitch_gains)
        if "throttle" in actions:
            fc.set_throttle(actions["throttle"])
        if "perturb_roll" in actions:
            fc.physics.apply_perturbation(roll_deg=actions["perturb_roll"])
        if "perturb_pitch" in actions:
            fc.physics.apply_perturbation(pitch_deg=actions["perturb_pitch"])
        if "scenario" in actions:
            idx = actions["scenario"]
            if logger: logger.stop(); logger = None
            scenario.start(idx)
            current_scene = SCENARIOS[idx]
            vis.reset_history(); m1=m2=m3=m4=cr=cp=0.0
            logger = FlightLogger(current_scene); logger.start()

        # Throttle manuel
        if fc.armed and scenario.active < 0:
            fc.set_throttle(panel.sl_throttle.value)

        # ── simulation ────────────────────────────────────────────
        steps = max(1, min(10, int(elapsed * SIM_HZ)))
        for _ in range(steps):
            scenario.tick(SIM_DT)
            roll, pitch, m1, m2, m3, m4, cr, cp = fc.update(SIM_DT)

        # ── état ──────────────────────────────────────────────────
        state = {
            "roll": fc.physics.roll, "pitch": fc.physics.pitch,
            "altitude": fc.physics.altitude, "vz": fc.physics.vz,
            "m1": m1, "m2": m2, "m3": m3, "m4": m4,
            "throttle": fc.base_throttle,
            "armed": fc.armed, "emergency": fc.emergency_stop,
            "corr_roll": cr, "corr_pitch": cp,
        }
        pid_terms = {
            "roll":  fc.get_pid_terms('roll'),
            "pitch": fc.get_pid_terms('pitch'),
        }

        vis.push_state(state)
        if logger and logger.is_running and fc.armed:
            logger.log(state, pid_terms)

        vis.draw(state, pid_terms, current_scene)
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