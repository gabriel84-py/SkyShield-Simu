"""
visualizer.py — Rendu Pygame de la simulation SkyShield

Trois zones :
  ┌──────────────────┬──────────────────┐
  │  Vue de face     │  Vue de dessus   │
  │  (roll + pitch)  │  (4 moteurs)     │
  ├──────────────────┴──────────────────┤
  │  Courbes temps réel                 │
  │  roll / pitch / altitude / moteurs  │
  └─────────────────────────────────────┘
"""

import pygame
import math
from collections import deque

# ─────────────────────────────────────────────────────────────────
# Palette couleurs
# ─────────────────────────────────────────────────────────────────
C_BG         = (15,  15,  25)      # fond général
C_PANEL      = (25,  25,  40)      # fond panneaux
C_GRID       = (40,  40,  60)      # grille graphes
C_WHITE      = (240, 240, 240)
C_GREY       = (120, 120, 140)
C_ROLL       = (33,  150, 243)     # bleu
C_PITCH      = (244, 67,  54)      # rouge
C_ALT        = (76,  175, 80)      # vert
C_M1         = (233, 30,  99)      # rose
C_M2         = (156, 39,  176)     # violet
C_M3         = (255, 152, 0)       # orange
C_M4         = (0,   188, 212)     # cyan
C_DRONE_BODY = (180, 180, 200)
C_DRONE_ARM  = (100, 120, 160)
C_WARNING    = (255, 180, 0)
C_DANGER     = (255, 60,  60)
C_THROTTLE   = (100, 220, 100)

HISTORY_LEN  = 400   # points dans les graphes (~4s à 100Hz)


# ─────────────────────────────────────────────────────────────────
class GraphPanel:
    """Trace une ou plusieurs courbes dans un rectangle Pygame."""

    def __init__(self, rect, y_min: float, y_max: float, title: str = ""):
        self.rect   = pygame.Rect(rect)
        self.y_min  = y_min
        self.y_max  = y_max
        self.title  = title
        self.series = []   # liste de (deque, couleur, label)

    def add_series(self, history: deque, color: tuple, label: str = ""):
        self.series.append((history, color, label))

    def draw(self, surface: pygame.Surface, font_small):
        # fond
        pygame.draw.rect(surface, C_PANEL, self.rect, border_radius=6)
        pygame.draw.rect(surface, C_GRID,  self.rect, 1, border_radius=6)

        # titre
        if self.title:
            txt = font_small.render(self.title, True, C_GREY)
            surface.blit(txt, (self.rect.x + 6, self.rect.y + 4))

        # ligne zéro
        if self.y_min < 0 < self.y_max:
            zy = self._y_to_px(0.0)
            pygame.draw.line(surface, C_GRID,
                             (self.rect.x, zy), (self.rect.right, zy), 1)

        # courbes
        for history, color, label in self.series:
            if len(history) < 2:
                continue
            pts = []
            n   = len(history)
            for i, val in enumerate(history):
                px = self.rect.x + int(i / (HISTORY_LEN - 1) * self.rect.width)
                py = self._y_to_px(val)
                pts.append((px, py))
            pygame.draw.lines(surface, color, False, pts, 2)

            # légende dernière valeur
            if label and history:
                last_val = history[-1]
                lbl = font_small.render(f"{label}: {last_val:+.1f}", True, color)
                # position dans la légende à droite
                idx = self.series.index((history, color, label))
                surface.blit(lbl, (self.rect.right - 110, self.rect.y + 4 + idx * 14))

    def _y_to_px(self, val: float) -> int:
        """Convertit une valeur → pixel Y dans le rectangle."""
        val   = max(self.y_min, min(self.y_max, val))
        ratio = (val - self.y_min) / (self.y_max - self.y_min)
        return int(self.rect.bottom - ratio * self.rect.height)


# ─────────────────────────────────────────────────────────────────
class Visualizer:
    """
    Fenêtre Pygame principale.
    Reçoit l'état du simulateur à chaque frame et l'affiche.
    """

    def __init__(self, width: int = 1100, height: int = 720):
        self.width  = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SkyShield Simulator — Trophées NSI 2026")

        self.font_large  = pygame.font.SysFont("monospace", 18, bold=True)
        self.font_medium = pygame.font.SysFont("monospace", 14)
        self.font_small  = pygame.font.SysFont("monospace", 11)

        # ── historiques des courbes ───────────────────────────────
        self.hist_roll    = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_pitch   = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_alt     = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_m1      = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_m2      = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_m3      = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_m4      = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_throttle= deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)

        # ── layout des panneaux ───────────────────────────────────
        panel_w = width // 2 - 10
        self._setup_panels(panel_w)

    # ─────────────────────────────────────────────────────────────
    def _setup_panels(self, panel_w: int):
        """Initialise les GraphPanel et leurs séries."""
        graph_y      = self.height // 2 + 10
        graph_h      = (self.height - graph_y - 10) // 2 - 5

        # graphe roll/pitch
        self.gp_angles = GraphPanel(
            (5, graph_y, self.width - 10, graph_h),
            y_min=-30.0, y_max=30.0, title="Roll & Pitch (°)"
        )
        self.gp_angles.add_series(self.hist_roll,  C_ROLL,  "Roll")
        self.gp_angles.add_series(self.hist_pitch, C_PITCH, "Pitch")

        # graphe moteurs
        self.gp_motors = GraphPanel(
            (5, graph_y + graph_h + 5, self.width - 10, graph_h),
            y_min=0.0, y_max=35.0, title="Throttle moteurs (%)"
        )
        self.gp_motors.add_series(self.hist_throttle, C_THROTTLE, "Base")
        self.gp_motors.add_series(self.hist_m1, C_M1, "M1")
        self.gp_motors.add_series(self.hist_m2, C_M2, "M2")
        self.gp_motors.add_series(self.hist_m3, C_M3, "M3")
        self.gp_motors.add_series(self.hist_m4, C_M4, "M4")

    # ─────────────────────────────────────────────────────────────
    def push_state(self, state: dict):
        """
        Ajoute un état dans les historiques.
        state doit contenir : roll, pitch, altitude, m1..m4, throttle
        """
        self.hist_roll.append(state.get("roll",     0.0))
        self.hist_pitch.append(state.get("pitch",   0.0))
        self.hist_alt.append(state.get("altitude",  0.0))
        self.hist_m1.append(state.get("m1",         0.0))
        self.hist_m2.append(state.get("m2",         0.0))
        self.hist_m3.append(state.get("m3",         0.0))
        self.hist_m4.append(state.get("m4",         0.0))
        self.hist_throttle.append(state.get("throttle", 0.0))

    # ─────────────────────────────────────────────────────────────
    def draw(self, state: dict, pid_terms: dict, scenario_name: str = ""):
        """
        Dessine une frame complète.
        state      : dict avec roll, pitch, altitude, vz, m1..m4, throttle, armed
        pid_terms  : dict avec p_roll, i_roll, d_roll, p_pitch, i_pitch, d_pitch
        """
        self.screen.fill(C_BG)

        half_w = self.width // 2

        # ── vue de face (gauche) ──────────────────────────────────
        self._draw_front_view(
            pygame.Rect(5, 5, half_w - 10, self.height // 2 - 10),
            state
        )

        # ── vue de dessus (droite) ────────────────────────────────
        self._draw_top_view(
            pygame.Rect(half_w + 5, 5, half_w - 10, self.height // 2 - 10),
            state
        )

        # ── graphes ───────────────────────────────────────────────
        self.gp_angles.draw(self.screen, self.font_small)
        self.gp_motors.draw(self.screen, self.font_small)

        # ── HUD texte ─────────────────────────────────────────────
        self._draw_hud(state, pid_terms, scenario_name)

        pygame.display.flip()

    # ─────────────────────────────────────────────────────────────
    def _draw_front_view(self, rect: pygame.Rect, state: dict):
        """Vue de face : drone représenté par ses bras inclinés selon roll."""
        pygame.draw.rect(self.screen, C_PANEL, rect, border_radius=8)
        pygame.draw.rect(self.screen, C_GRID,  rect, 1, border_radius=8)

        title = self.font_medium.render("Vue de face — Roll / Pitch", True, C_GREY)
        self.screen.blit(title, (rect.x + 8, rect.y + 6))

        cx = rect.centerx
        cy = rect.centery + 10

        roll_deg  = state.get("roll",  0.0)
        pitch_deg = state.get("pitch", 0.0)
        alt       = state.get("altitude", 0.0)

        # ── sol ───────────────────────────────────────────────────
        ground_y = rect.bottom - 20
        pygame.draw.line(self.screen, C_GRID,
                         (rect.x + 10, ground_y), (rect.right - 10, ground_y), 2)
        sol_lbl = self.font_small.render("SOL", True, C_GRID)
        self.screen.blit(sol_lbl, (rect.x + 10, ground_y + 2))

        # ── position verticale du drone ───────────────────────────
        max_alt_display = 3.0   # m → hauteur visuelle max
        alt_px = int(min(alt / max_alt_display, 1.0) * (ground_y - rect.y - 80))
        drone_y = ground_y - alt_px - 20

        # ── bras du drone (inclinés selon roll) ───────────────────
        arm_len = 55
        roll_rad = math.radians(roll_deg)

        # Bras gauche et droit (symétrie axe roll)
        dx_left  = -arm_len * math.cos(roll_rad)
        dy_left  = -arm_len * math.sin(roll_rad)  # positif = monte
        dx_right =  arm_len * math.cos(roll_rad)
        dy_right =  arm_len * math.sin(roll_rad)

        # pitch : incline légèrement le drone vers l'avant/arrière (affiché en bas)
        # On visualise le pitch par un indicateur séparé

        p_left  = (int(cx + dx_left),  int(drone_y - dy_left))
        p_right = (int(cx + dx_right), int(drone_y - dy_right))

        # corps
        pygame.draw.circle(self.screen, C_DRONE_BODY, (cx, int(drone_y)), 10)

        # bras
        pygame.draw.line(self.screen, C_DRONE_ARM, (cx, int(drone_y)), p_left,  4)
        pygame.draw.line(self.screen, C_DRONE_ARM, (cx, int(drone_y)), p_right, 4)

        # moteurs aux extrémités
        m_colors = [C_M1, C_M4]  # front-right, rear-right (vue de face simplifiée)
        throttles = [state.get("m1", 0.0), state.get("m4", 0.0)]
        for pt, color, thr in zip([p_right, p_left], m_colors, throttles):
            r = 10 + int(thr / 100.0 * 8)   # rayon proportionnel au throttle
            pygame.draw.circle(self.screen, color, pt, r)
            glow_alpha = min(255, int(thr / 30.0 * 180))
            if glow_alpha > 20:
                glow_surf = pygame.Surface((r*4, r*4), pygame.SRCALPHA)
                pygame.draw.circle(glow_surf, (*color, glow_alpha), (r*2, r*2), r*2)
                self.screen.blit(glow_surf, (pt[0] - r*2, pt[1] - r*2))

        # ── indicateur pitch (arc) ────────────────────────────────
        pitch_bar_x = rect.right - 30
        pitch_bar_top = rect.y + 30
        pitch_bar_h = rect.height - 60
        pitch_pct = (pitch_deg + 30) / 60.0
        pitch_pct = max(0.0, min(1.0, pitch_pct))
        bar_y_fill = int(pitch_bar_top + (1 - pitch_pct) * pitch_bar_h)
        pygame.draw.rect(self.screen, C_GRID,  (pitch_bar_x, pitch_bar_top, 12, pitch_bar_h))
        pygame.draw.rect(self.screen, C_PITCH, (pitch_bar_x, bar_y_fill, 12, pitch_bar_top + pitch_bar_h - bar_y_fill))
        lbl = self.font_small.render("P", True, C_PITCH)
        self.screen.blit(lbl, (pitch_bar_x + 1, pitch_bar_top - 14))

        # ── altitude texte ────────────────────────────────────────
        alt_color = C_ALT if alt > 0.05 else C_GREY
        alt_txt = self.font_medium.render(f"Alt : {alt:.2f} m", True, alt_color)
        self.screen.blit(alt_txt, (rect.x + 8, rect.bottom - 22))

        roll_color = C_WARNING if abs(roll_deg) > 20 else C_ROLL
        if abs(roll_deg) > 40:
            roll_color = C_DANGER
        roll_txt = self.font_medium.render(f"Roll: {roll_deg:+.1f}°", True, roll_color)
        self.screen.blit(roll_txt, (rect.x + 8, rect.bottom - 40))

    # ─────────────────────────────────────────────────────────────
    def _draw_top_view(self, rect: pygame.Rect, state: dict):
        """Vue de dessus : 4 moteurs avec intensité lumineuse proportionnelle au throttle."""
        pygame.draw.rect(self.screen, C_PANEL, rect, border_radius=8)
        pygame.draw.rect(self.screen, C_GRID,  rect, 1, border_radius=8)

        title = self.font_medium.render("Vue de dessus — Moteurs", True, C_GREY)
        self.screen.blit(title, (rect.x + 8, rect.y + 6))

        cx = rect.centerx
        cy = rect.centery + 5
        arm = 65

        # positions des 4 moteurs (config X)
        motor_pos = {
            "M1": (cx + arm, cy - arm),   # front-right
            "M2": (cx - arm, cy - arm),   # front-left
            "M3": (cx - arm, cy + arm),   # rear-left
            "M4": (cx + arm, cy + arm),   # rear-right
        }
        motor_colors = {
            "M1": C_M1, "M2": C_M2, "M3": C_M3, "M4": C_M4
        }
        motor_throttles = {
            "M1": state.get("m1", 0.0),
            "M2": state.get("m2", 0.0),
            "M3": state.get("m3", 0.0),
            "M4": state.get("m4", 0.0),
        }
        motor_dir = {
            "M1": "CW", "M2": "CCW", "M3": "CW", "M4": "CCW"
        }

        # ── bras ─────────────────────────────────────────────────
        pygame.draw.line(self.screen, C_DRONE_ARM,
                         motor_pos["M1"], motor_pos["M3"], 3)
        pygame.draw.line(self.screen, C_DRONE_ARM,
                         motor_pos["M2"], motor_pos["M4"], 3)

        # ── corps central ─────────────────────────────────────────
        pygame.draw.circle(self.screen, C_DRONE_BODY, (cx, cy), 14)

        # flèche avant
        pygame.draw.polygon(self.screen, C_WHITE, [
            (cx, cy - 20), (cx - 6, cy - 10), (cx + 6, cy - 10)
        ])

        # ── moteurs ───────────────────────────────────────────────
        for name, pos in motor_pos.items():
            thr    = motor_throttles[name]
            color  = motor_colors[name]
            radius = 14

            # halo proportionnel au throttle
            if thr > 0:
                halo_r = int(radius + thr / 100.0 * 20)
                alpha  = min(200, int(thr / 30.0 * 160))
                halo   = pygame.Surface((halo_r * 2, halo_r * 2), pygame.SRCALPHA)
                pygame.draw.circle(halo, (*color, alpha), (halo_r, halo_r), halo_r)
                self.screen.blit(halo, (pos[0] - halo_r, pos[1] - halo_r))

            pygame.draw.circle(self.screen, color, pos, radius)
            pygame.draw.circle(self.screen, C_PANEL, pos, radius - 3)

            # pourcentage throttle
            thr_txt = self.font_small.render(f"{thr:.0f}%", True, color)
            self.screen.blit(thr_txt, (pos[0] - 14, pos[1] + radius + 2))

            # nom moteur + sens
            n_txt = self.font_small.render(f"{name} {motor_dir[name]}", True, C_GREY)
            self.screen.blit(n_txt, (pos[0] - 22, pos[1] - radius - 14))

        # ── indicateurs roll / pitch ──────────────────────────────
        roll  = state.get("roll",  0.0)
        pitch = state.get("pitch", 0.0)
        r_col = C_WARNING if abs(roll)  > 20 else C_ROLL
        p_col = C_WARNING if abs(pitch) > 20 else C_PITCH

        self.screen.blit(self.font_small.render(f"Roll : {roll:+.1f}°",  True, r_col),
                         (rect.x + 8, rect.bottom - 40))
        self.screen.blit(self.font_small.render(f"Pitch: {pitch:+.1f}°", True, p_col),
                         (rect.x + 8, rect.bottom - 24))

    # ─────────────────────────────────────────────────────────────
    def _draw_hud(self, state: dict, pid_terms: dict, scenario_name: str):
        """Overlay HUD : valeurs numériques, état armement, scénario actif."""
        armed   = state.get("armed", False)
        em_stop = state.get("emergency", False)

        # ── état armement ─────────────────────────────────────────
        if em_stop:
            status_txt = "⚠ EMERGENCY STOP"
            status_col = C_DANGER
        elif armed:
            status_txt = "● ARMÉ"
            status_col = C_ALT
        else:
            status_txt = "○ DÉSARMÉ"
            status_col = C_GREY

        s = self.font_large.render(status_txt, True, status_col)
        self.screen.blit(s, (self.width - s.get_width() - 10, 8))

        # ── scénario actif ────────────────────────────────────────
        if scenario_name:
            sc = self.font_medium.render(f"Scénario : {scenario_name}", True, C_WHITE)
            self.screen.blit(sc, (self.width // 2 - sc.get_width() // 2, 8))

        # ── termes PID ────────────────────────────────────────────
        pr, ir, dr = pid_terms.get("roll",  (0, 0, 0))
        pp, ip, dp = pid_terms.get("pitch", (0, 0, 0))

        lines = [
            ("PID Roll",  f"P={pr:+.2f}  I={ir:+.2f}  D={dr:+.2f}", C_ROLL),
            ("PID Pitch", f"P={pp:+.2f}  I={ip:+.2f}  D={dp:+.2f}", C_PITCH),
        ]
        for idx, (label, val, col) in enumerate(lines):
            lbl = self.font_small.render(label, True, col)
            val_s = self.font_small.render(val, True, C_WHITE)
            y = self.height // 2 - 44 + idx * 16
            self.screen.blit(lbl,  (self.width // 2 - 240, y))
            self.screen.blit(val_s,(self.width // 2 - 160, y))

    # ─────────────────────────────────────────────────────────────
    def reset_history(self):
        for h in [self.hist_roll, self.hist_pitch, self.hist_alt,
                  self.hist_m1, self.hist_m2, self.hist_m3, self.hist_m4,
                  self.hist_throttle]:
            h.clear()
            h.extend([0.0] * HISTORY_LEN)