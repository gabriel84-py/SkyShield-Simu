# Algorithme PID — SkyShield

## Pourquoi un PID ?

Un drone quadrirotor est un système **naturellement instable** : sans correction active,
le moindre déséquilibre (vent, dissymétrie des moteurs) le fait basculer. Le PID est
l'algorithme de référence dans l'industrie des drones pour sa simplicité, sa robustesse
et son faible coût en calcul — essentiel sur microcontrôleur.

## Principe

Le PID mesure en permanence l'écart entre l'angle cible (setpoint = 0°, drone à plat)
et l'angle réel mesuré par le gyroscope (ou la physique simulée), et calcule une
correction à appliquer aux moteurs.

```
erreur = setpoint - mesure

sortie = Kp × erreur
       + Ki × ∫ erreur dt
       + Kd × d(erreur)/dt
```

## Les trois termes

**Terme P (Proportionnel)** — réagit à l'erreur instantanée.
Un drone penché à 10° reçoit une correction proportionnelle à 10°.
Trop grand → oscillations. Trop petit → correction lente.

**Terme I (Intégral)** — corrige les erreurs persistantes.
Si le drone dérive lentement sans revenir à 0°, l'intégrale s'accumule et force la
correction. Dans notre cas Ki=0 : le drone simulé est parfaitement symétrique, pas de
dérive constante à corriger.

**Terme D (Dérivé)** — anticipe en freinant les mouvements rapides.
Si l'erreur diminue vite (le drone revient vers 0°), le terme D réduit la correction
pour éviter le dépassement. C'est le terme qui "amorti" le système.

## Nos gains et leur justification

| Gain | Valeur | Raison |
|---|---|---|
| Kp | 0.8 | Déterminé empiriquement sur le vrai drone — réponse rapide sans oscillation |
| Ki | 0.0 | Pas de dérive mesurée, risque de windup sans bénéfice |
| Kd | 0.25 | Amorti suffisant pour compenser le délai moteur (MOTOR_TAU=0.07s) |

## Démonstration : scénario 3 (PID oscillant)

Avec Kp=2.5 et Kd=0, le système oscille à ~28°. L'explication :

1. Le drone est perturbé à +12° de roll.
2. Le terme P envoie une correction forte (2.5 × 12 = 30%).
3. Sans terme D, rien ne freine la correction — le drone dépasse 0° dans l'autre sens.
4. Le terme P corrige dans l'autre sens, trop fort à nouveau.
5. Le filtre moteur (MOTOR_TAU) ajoute un délai qui amplifie le phénomène.

→ Oscillations stables à ~28°, non fatales car le safety stop n'intervient qu'à 60°.

## Protections implémentées

**Anti-windup** : l'intégrale est bornée à ±10. Sans ce clamp, une perturbation longue
ferait exploser l'intégrale, provoquant une sur-correction brutale au retour à l'équilibre.

**Clamp dérivée** : la dérivée brute est limitée à ±500°/s avant multiplication par Kd.
Évite les pics ("derivative kick") lors de changements brusques de setpoint.

**Protection dt** : si le pas de temps est ≤0 ou >0.5s (lag système, premier appel),
on substitue dt=0.01s. Évite une dérivée infinie ou nulle au démarrage.

**Reset** : l'intégrale et la dernière erreur sont remises à zéro à chaque armement et
à chaque coupure des gaz. Évite qu'un résidu d'intégrale d'un vol précédent perturbe
le décollage suivant.

## Un PID par axe

Deux instances PID indépendantes tournent en parallèle : une pour le roll, une pour le
pitch. Elles partagent les mêmes gains dans nos scénarios, mais peuvent être réglées
séparément via les sliders de l'interface — ce qui correspond au comportement réel d'un
contrôleur de vol professionnel.