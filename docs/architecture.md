# Architecture — SkyShield

## Vue d'ensemble

```
main.py
  │
  ├── Scenario          ← gère les scénarios automatiques (perturbations, rampes)
  │
  ├── FlightControllerSim   ← contrôleur de vol
  │     ├── PID (roll)        ← algorithme de stabilisation axe roll
  │     ├── PID (pitch)       ← algorithme de stabilisation axe pitch
  │     └── DronePhysics      ← modèle physique du drone
  │
  ├── Visualizer        ← rendu Pygame (vues 2D + graphes temps réel)
  ├── ControlPanel      ← interface utilisateur (sliders, boutons)
  └── FlightLogger      ← enregistrement CSV
```

## Flux de données par frame (100 Hz)

```
ControlPanel  →  throttle, gains PID, perturbations
      ↓
Scenario.tick()  →  perturbations automatiques sur DronePhysics
      ↓
FlightControllerSim.update()
      ├── lit roll, pitch  ←  DronePhysics
      ├── PID.compute()    →  corrections corr_roll, corr_pitch
      ├── _mix()           →  consignes m1, m2, m3, m4
      └── DronePhysics.step(m1..m4, dt)  →  intègre la physique
      ↓
état : roll, pitch, altitude, vz, m1..m4
      ↓
  ┌───┴───┐
Visualizer  FlightLogger
(affichage) (CSV)
```

## Responsabilités de chaque module

**`pid.py`** — Algorithme pur, sans dépendance. Prend (setpoint, mesure), retourne une correction. Identique au code embarqué sur le Pico RP2040.

**`physics.py`** — Modèle physique isolé. Prend les 4 consignes moteur, intègre les équations du mouvement, met à jour roll/pitch/altitude. Ne connaît pas le PID.

**`flight_controller_sim.py`** — Fait le lien entre PID et physique. Gère l'armement, le mixage des moteurs, les seuils de sécurité (safety stop à 60°), le warmup (80 steps d'immunité au démarrage).

**`main.py`** — Boucle principale 60 FPS. Orchestre tous les modules, gère les événements clavier, lance les scénarios, pilote le logger.

**`visualizer.py`** — Lecture seule de l'état. Ne modifie jamais la simulation.

**`ui.py`** — Produit un dictionnaire d'actions (`throttle`, `pid_changed`, `perturb_roll`…) que `main.py` applique. Découplé du contrôleur.

**`logger.py`** — Écrit un CSV par vol dans `data/`. Flush toutes les 100 lignes.

## Choix de conception

**Séparation physique / contrôleur** : `DronePhysics` ne sait pas qu'il y a un PID. Cela permet de tester le moteur physique indépendamment et de brancher n'importe quel contrôleur (PID, bang-bang, manuel) sans modifier la physique.

**Miroir du code embarqué** : `flight_controller_sim.py` et `pid.py` sont fonctionnellement identiques à leurs équivalents MicroPython du vrai drone. Seules les APIs temps réel changent (`time.ticks_us` → `time.perf_counter`).

**Fréquence découplée** : la simulation tourne à 100 Hz, l'affichage à 60 FPS. `main.py` calcule le nombre de steps à exécuter par frame pour rester synchrone même si le rendu ralentit.