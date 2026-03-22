# Modèle physique — SkyShield

## Paramètres du vrai drone

Toutes les constantes du modèle sont issues du drone réel, pas estimées arbitrairement.

| Paramètre | Valeur | Origine                                                 |
|---|---|---------------------------------------------------------|
| Masse | 444 g | Pesée de l'ensemble (frame + moteurs + batterie + Pico) |
| Frame | 250mm True X | Référence du châssis imprimé en 3D                      |
| Longueur de bras | 125 mm | Demi-diagonale de la frame (250/2)                      |
| Hover throttle | 20% | Calibration ESC : seuil mesuré expérimentalement        |
| THRUST_MAX | 21.78 N | poids(4.356N) / (20/100)                                |
| Poussée par moteur | 5.44 N | THRUST_MAX / 4                                          |
| Inertie roll/pitch | 0.003 kg·m² | Estimation géométrique (masse × bras²)                  |

## Axes et convention de signes

```
        AVANT
    M2(CCW)  M1(CW)
       \      /
        \ ↑ /
   ←     corps     →
        / ↓ \
       /      \
    M3(CW)  M4(CCW)
        ARRIÈRE

Roll  > 0 → drone penche à droite
Pitch > 0 → drone penche en avant
Z     > 0 → drone monte
```

## Équations du mouvement

### Dynamique angulaire (roll et pitch)

Les 4 forces moteur génèrent des couples autour des axes roll et pitch :

```
torque_roll  = (F2 + F3 - F1 - F4) × L
torque_pitch = (F1 + F2 - F3 - F4) × L
```

avec L = longueur de bras (0.125m), Fi = force du moteur i en Newtons.

L'accélération angulaire s'en déduit (2e loi de Newton en rotation) :

```
α_roll  = (torque_roll  - drag × ω_roll)  / I
α_pitch = (torque_pitch - drag × ω_pitch) / I
```

avec I = inertie (0.003 kg·m²) et drag = amortissement naturel (0.05 N·m·s/rad).

Intégration Euler à pas fixe (dt = 0.01s) :
```
ω += α × dt
θ += ω × dt
```

### Dynamique verticale

La poussée effective dépend de l'inclinaison du drone :

```
F_verticale = (F1+F2+F3+F4) × cos(roll) × cos(pitch)
F_nette     = F_verticale - poids - drag_vertical × vz
vz         += (F_nette / masse) × dt
altitude   += vz × dt
```

Le facteur `cos(roll)×cos(pitch)` est crucial : un drone incliné à 45° perd ~30% de
sa poussée verticale.

## Le filtre passe-bas moteur

C'est l'élément clé de la fidélité de la simulation.

Les moteurs réels ne répondent pas instantanément à une consigne : il y a un délai dû
à l'inertie mécanique du moteur et au temps de réponse de l'ESC. On le modélise par un
filtre passe-bas du premier ordre :

```
a = dt / (dt + MOTOR_TAU)
m_filtré += a × (m_consigne - m_filtré)
```

avec MOTOR_TAU = 0.07s (mesuré empiriquement sur nos ESC).

**Pourquoi c'est important :** sans ce filtre, le PID reçoit une réponse instantanée des
moteurs et reste stable même avec de mauvais gains. Avec le filtre, le délai crée un
déphasage qui provoque des oscillations avec un Kd=0 — exactement ce qu'on observait sur
le vrai drone. Le scénario 3 n'aurait pas de sens sans MOTOR_TAU.

## Seuils et sécurités

**MIN_MOTOR = 10%** : en dessous, le moteur ne tourne pas (seuil de démarrage réel des
moteurs brushless). Les forces sont nulles sous ce seuil.

**Clamp moteurs à 30%** : les consignes sont bornées à [0, 30%]. Sans ce clamp,
throttle=22% + correction=25% → moteur à 47%, soit ×2.35 la force attendue, ce qui
créerait un couple parasite et ferait diverger le pitch.

**Safety stop à 60°** : si roll ou pitch dépasse 60° par rapport à la cible, le
contrôleur déclenche un arrêt d'urgence. En pratique, les oscillations du scénario 3
culminent à ~28° — bien en dessous du seuil.

**Warmup (80 steps)** : le safety stop est désactivé pendant les 80 premiers pas (0.8s)
pour éviter un faux positif au démarrage, quand les angles initialisés à 0° peuvent
brièvement dépasser le seuil à cause du transitoire PID.

## Limites du modèle

- **Pas de yaw** : le couple de réaction des moteurs (CW/CCW) n'est pas modélisé.
- **Vent simplifié** : injection directe d'un décalage d'angle, pas de force continue.
- **Inertie approchée** : I=0.003 kg·m² est une estimation — une mesure expérimentale
  (oscillations libres) donnerait une valeur plus précise.
- **Euler diverge pour grands angles** : l'intégration Euler est exacte pour de petits
  angles. Le safety stop à 60° évite ce régime en pratique.