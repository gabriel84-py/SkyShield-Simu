# SkyShield
## Sidonie Albert, Gabriel Jean Vermeille

## Utilisation
### Installation :
```bash
pip install -r requirements.txt
```
### Démarrage du simulateur :
```bash
python3 sources/main.py
```
### Utilisation du simulateur : 
```text
Scénarios :
  1 — Stabilisation (throttle 22%, hover=20%)
  2 — Perturbation Roll (décalage d'angle ±15° toutes les 4s)
  3 — PID oscillant (Kp=2.5, Kd=0 → oscillations ~28°)
  4 — Décollage progressif (rampe 0→24% en 5s)

Raccourcis :
  ESPACE  : ARM / DISARM     R : Reset
  1-4     : Scénarios        ↑/↓ : Throttle ±1%
  ECHAP   : Quitter
```


## Structure
- `data/` - Logs de chaque simulation
- `sources/flight_controller_sim.py` - Controleur de vol simulé
- `sources/logger.py` - Génération des logs
- `sources/physics.py` - Modélisation des conditions réelles physiques
- `sources/pid.py` - Algorithme de stabilisation P,I et D
- `sources/main.py` - Script principal du simulateur
- `sources/ui.py` - Interface utilisateur en pygame
- `sources/visualizer.py` - Rendu simulateur en pygame


## Répartition du travail
 
- **Gabriel** : algorithme PID (`pid.py`), moteur physique (`physics.py`), contrôleur de vol (`flight_controller_sim.py`), logique des scénarios (`main.py`).
- **Sidonie** : interface utilisateur (`ui.py`), visualiseur (`visualizer.py`), logger (`logger.py`).
