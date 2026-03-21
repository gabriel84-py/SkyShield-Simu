"""
-Sido
ui.py — Interface utilisateur Pygame pour SkyShield Simulator

Panneau latéral rétractable avec :
  - Sliders Kp / Ki / Kd pour Roll et Pitch
  - Boutons : Perturbation Roll/Pitch, Reset, ARM/DISARM
  - Sélecteur de scénario
  - Affichage throttle live
"""

import pygame

# ─────────────────────────────────────────────────────────────────
# Couleurs (cohérentes avec visualizer.py)
# ─────────────────────────────────────────────────────────────────
C_BG        = (15,  15,  25)
C_PANEL     = (30,  30,  48)
C_BORDER    = (60,  60,  90)
C_ACCENT    = (90,  120, 200)
C_WHITE     = (240, 240, 240)
C_GREY      = (120, 120, 140)
C_ROLL      = (33,  150, 243)
C_PITCH     = (244, 67,  54)
C_GREEN     = (76,  175, 80)
C_ORANGE    = (255, 152, 0)
C_RED       = (220, 50,  50)
C_DARK_RED  = (140, 20,  20)
C_SLIDER_BG = (50,  50,  75)
C_SLIDER_FG = (90,  130, 220)
C_BTN_NORM  = (45,  55,  85)
C_BTN_HOV   = (65,  80,  130)
C_BTN_ACT   = (30,  160, 100)


# ─────────────────────────────────────────────────────────────────
class Slider:
    """Slider horizontal Pygame."""

    def __init__(self, x, y, w, label: str,
                 val_min: float, val_max: float, val_init: float,
                 color=C_SLIDER_FG, step: float = 0.01):
        self.rect    = pygame.Rect(x, y, w, 6)
        self.label   = label
        self.val_min = val_min
        self.val_max = val_max
        self.value   = val_init
        self.color   = color
        self.step    = step
        self.dragging = False

    @property
    def handle_x(self) -> int:
        ratio = (self.value - self.val_min) / (self.val_max - self.val_min)
        return int(self.rect.x + ratio * self.rect.width)

    def handle_rect(self) -> pygame.Rect:
        return pygame.Rect(self.handle_x - 7, self.rect.y - 6, 14, 18)

    def draw(self, surface, font_small):
        # piste
        pygame.draw.rect(surface, C_SLIDER_BG, self.rect, border_radius=3)
        # remplissage
        fill_w = self.handle_x - self.rect.x
        if fill_w > 0:
            pygame.draw.rect(surface, self.color,
                             (self.rect.x, self.rect.y, fill_w, self.rect.height),
                             border_radius=3)
        # poignée
        h_col = C_WHITE if self.dragging else C_GREY
        pygame.draw.rect(surface, h_col, self.handle_rect(), border_radius=4)

        # label + valeur
        txt = font_small.render(f"{self.label}: {self.value:.3f}", True, C_WHITE)
        surface.blit(txt, (self.rect.x, self.rect.y - 16))

    def handle_event(self, event) -> bool:
        """Retourne True si la valeur a changé."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.handle_rect().collidepoint(event.pos) or \
               self.rect.inflate(0, 20).collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            mx = event.pos[0]
            ratio = (mx - self.rect.x) / self.rect.width
            ratio = max(0.0, min(1.0, ratio))
            new_val = self.val_min + ratio * (self.val_max - self.val_min)
            # arrondi au step
            new_val = round(new_val / self.step) * self.step
            if abs(new_val - self.value) > 1e-6:
                self.value = new_val
                return True
        return False


# ─────────────────────────────────────────────────────────────────
class Button:
    """Bouton cliquable simple."""

    def __init__(self, x, y, w, h, label: str,
                 color=C_BTN_NORM, active_color=C_BTN_ACT):
        self.rect         = pygame.Rect(x, y, w, h)
        self.label        = label
        self.color        = color
        self.active_color = active_color
        self.hovered      = False
        self.pressed      = False    # état toggle si besoin

    def draw(self, surface, font_medium):
        col = self.active_color if self.pressed else \
              (C_BTN_HOV if self.hovered else self.color)
        pygame.draw.rect(surface, col, self.rect, border_radius=5)
        pygame.draw.rect(surface, C_BORDER, self.rect, 1, border_radius=5)
        txt = font_medium.render(self.label, True, C_WHITE)
        tx  = self.rect.centerx - txt.get_width()  // 2
        ty  = self.rect.centery - txt.get_height() // 2
        surface.blit(txt, (tx, ty))

    def handle_event(self, event) -> bool:
        """Retourne True si le bouton vient d'être cliqué."""
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False


# ─────────────────────────────────────────────────────────────────
class ControlPanel:
    """
    Panneau de contrôle latéral (côté droit de la fenêtre simulateur).

    Contient :
      - 6 sliders PID (Kp/Ki/Kd × Roll et Pitch)
      - Boutons : ARM, DISARM, RESET
      - Boutons perturbation : Roll+, Roll-, Pitch+, Pitch-
      - Sélecteur scénario
      - Affichage throttle (slider lecture/écriture)
    """

    PANEL_W = 280

    def __init__(self, screen_w: int, screen_h: int):
        self.x = screen_w - self.PANEL_W
        self.y = 0
        self.w = self.PANEL_W
        self.h = screen_h

        self.font_medium = pygame.font.SysFont("monospace", 13, bold=True)
        self.font_small  = pygame.font.SysFont("monospace", 11)

        self._build(screen_w, screen_h)

    # ─────────────────────────────────────────────────────────────
    def _build(self, sw, sh):
        x0 = self.x + 14
        y  = 30

        def s(label, vmin, vmax, vinit, color, step=0.01):
            nonlocal y
            sl = Slider(x0, y + 18, self.w - 28, label, vmin, vmax, vinit, color, step)
            y += 46
            return sl

        # ── section Roll ──────────────────────────────────────────
        self._roll_section_y = y
        y += 18
        self.sl_roll_kp = s("Kp Roll", 0.0, 5.0, 0.8,  C_ROLL, 0.01)
        self.sl_roll_ki = s("Ki Roll", 0.0, 1.0, 0.0,  C_ROLL, 0.001)
        self.sl_roll_kd = s("Kd Roll", 0.0, 2.0, 0.25, C_ROLL, 0.005)

        # ── section Pitch ─────────────────────────────────────────
        y += 8
        self._pitch_section_y = y
        y += 18
        self.sl_pitch_kp = s("Kp Pitch", 0.0, 5.0, 0.8,  C_PITCH, 0.01)
        self.sl_pitch_ki = s("Ki Pitch", 0.0, 1.0, 0.0,  C_PITCH, 0.001)
        self.sl_pitch_kd = s("Kd Pitch", 0.0, 2.0, 0.25, C_PITCH, 0.005)

        # ── throttle ──────────────────────────────────────────────
        y += 8
        self._thr_section_y = y
        y += 18
        self.sl_throttle = Slider(x0, y + 18, self.w - 28,
                                  "Throttle %", 0.0, 30.0, 0.0, C_GREEN, 0.5)
        y += 50

        # ── boutons contrôle ──────────────────────────────────────
        y += 8
        bw, bh = (self.w - 28) // 2 - 4, 30
        self.btn_arm    = Button(x0,       y, bw, bh, "ARM",    C_BTN_NORM, C_BTN_ACT)
        self.btn_disarm = Button(x0+bw+8,  y, bw, bh, "DISARM", C_BTN_NORM, C_DARK_RED)
        y += bh + 8

        self.btn_reset  = Button(x0, y, self.w - 28, bh, "RESET", C_ORANGE)
        y += bh + 14

        # ── boutons perturbation ──────────────────────────────────
        self._perturb_y = y
        y += 18
        pw = (self.w - 28) // 2 - 4
        self.btn_roll_p  = Button(x0,      y, pw, bh, "Roll +30°", C_BTN_NORM)
        self.btn_roll_n  = Button(x0+pw+8, y, pw, bh, "Roll -30°", C_BTN_NORM)
        y += bh + 6
        self.btn_pitch_p = Button(x0,      y, pw, bh, "Pitch +30°", C_BTN_NORM)
        self.btn_pitch_n = Button(x0+pw+8, y, pw, bh, "Pitch -30°", C_BTN_NORM)
        y += bh + 14

        # ── scénarios ─────────────────────────────────────────────
        self._scenario_y = y
        y += 18
        self.scenarios = [
            "Stabilisation",
            "Perturbation Roll",
            "PID oscillant",
            "Décollage",
        ]
        self.active_scenario = 0
        self._scenario_btns = []
        for i, name in enumerate(self.scenarios):
            btn = Button(x0, y, self.w - 28, 26, name)
            self._scenario_btns.append(btn)
            y += 30

        self.all_sliders = [
            self.sl_roll_kp, self.sl_roll_ki, self.sl_roll_kd,
            self.sl_pitch_kp, self.sl_pitch_ki, self.sl_pitch_kd,
            self.sl_throttle,
        ]
        self.all_buttons = [
            self.btn_arm, self.btn_disarm, self.btn_reset,
            self.btn_roll_p, self.btn_roll_n,
            self.btn_pitch_p, self.btn_pitch_n,
        ] + self._scenario_btns

    # ─────────────────────────────────────────────────────────────
    def draw(self, surface: pygame.Surface):
        # fond panneau
        pygame.draw.rect(surface, C_PANEL,
                         (self.x, self.y, self.w, self.h))
        pygame.draw.line(surface, C_BORDER,
                         (self.x, 0), (self.x, self.h), 1)

        def section(label, y, color):
            txt = self.font_medium.render(f"── {label} ──", True, color)
            surface.blit(txt, (self.x + 14, y))

        section("PID ROLL",       self._roll_section_y,    C_ROLL)
        section("PID PITCH",      self._pitch_section_y,   C_PITCH)
        section("THROTTLE",       self._thr_section_y,     C_GREEN)
        section("PERTURBATIONS",  self._perturb_y,         C_ORANGE)
        section("SCÉNARIOS",      self._scenario_y,        C_GREY)

        for sl in self.all_sliders:
            sl.draw(surface, self.font_small)

        for btn in self.all_buttons:
            # marque le scénario actif
            if btn in self._scenario_btns:
                btn.pressed = (self._scenario_btns.index(btn) == self.active_scenario)
            btn.draw(surface, self.font_medium)

    # ─────────────────────────────────────────────────────────────
    def handle_events(self, events: list) -> dict:
        """
        Traite les événements Pygame et retourne un dict d'actions.

        Clés possibles du dict retourné :
          "pid_changed"   : True si un gain PID a changé
          "throttle"      : float, nouvelle valeur throttle
          "arm"           : True
          "disarm"        : True
          "reset"         : True
          "perturb_roll"  : float (±30)
          "perturb_pitch" : float (±30)
          "scenario"      : int, index du scénario sélectionné
        """
        actions = {}

        for event in events:
            # ── sliders PID ───────────────────────────────────────
            for sl in [self.sl_roll_kp, self.sl_roll_ki, self.sl_roll_kd,
                       self.sl_pitch_kp, self.sl_pitch_ki, self.sl_pitch_kd]:
                if sl.handle_event(event):
                    actions["pid_changed"] = True

            # ── throttle ──────────────────────────────────────────
            if self.sl_throttle.handle_event(event):
                actions["throttle"] = self.sl_throttle.value

            # ── boutons ───────────────────────────────────────────
            if self.btn_arm.handle_event(event):
                actions["arm"] = True
            if self.btn_disarm.handle_event(event):
                actions["disarm"] = True
            if self.btn_reset.handle_event(event):
                actions["reset"] = True

            if self.btn_roll_p.handle_event(event):
                actions["perturb_roll"] = 30.0
            if self.btn_roll_n.handle_event(event):
                actions["perturb_roll"] = -30.0
            if self.btn_pitch_p.handle_event(event):
                actions["perturb_pitch"] = 30.0
            if self.btn_pitch_n.handle_event(event):
                actions["perturb_pitch"] = -30.0

            for i, btn in enumerate(self._scenario_btns):
                if btn.handle_event(event):
                    self.active_scenario = i
                    actions["scenario"] = i

        return actions

    # ─────────────────────────────────────────────────────────────
    @property
    def roll_gains(self) -> tuple:
        return self.sl_roll_kp.value, self.sl_roll_ki.value, self.sl_roll_kd.value

    @property
    def pitch_gains(self) -> tuple:
        return self.sl_pitch_kp.value, self.sl_pitch_ki.value, self.sl_pitch_kd.value

    @property
    def throttle(self) -> float:
        return self.sl_throttle.value