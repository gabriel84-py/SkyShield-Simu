# Présentation

## Historique du projet

Initialement, nous avions comme projet de créer un drone parapluie autonome. Vos mains enfin libérées sous la pluie.
Un projet certes ambitieux, mais qui nous paraissait réalisable. 
Et on y était presque : modèle d'intelligence artificielle codé par nos soins, algorithme de stabilisation, contrôleur
de vol en état de marche, Raspberry Pi et sa caméra optimisés pour la vision par ordinateur, drone fonctionnel !

Pour cela nous avons soudé, codé, configuré, gambergé, optimisé et stabilisé pendant des heures. Chaque étape
minutieusement documentée par de nombreuses photos et vidéos.

Nous avions presque réussi à coder de zéro un drone autonome. Il ne nous restait qu'à assembler les pièces du puzzle.

Mais, le 27 février à 17h30, lors d'un test de calibration des ESC, le drame. La masse et la data d'une des ESC se 
connectent par erreur, le moteur avant droit s'emballe, le drone décolle à pleine puissance et, en une fraction de seconde,
se crashe dans le mur. La batterie, deux des quatre moteurs, les quatre ESC et le microcontroleur (notre controleur de vol
maison) sont détruits.

Il ne nous reste plus qu'un mois pour soumettre notre projet et des composants essentiels tels que le ESC ne sont plus disponibles
chez nos fournisseurs. Nous n'avons plus de drone. Les seuls vestiges de notre projet sont un dossier de photos et vidéos 
et notre code.

Après de longues réflexions, alors que le temps nous était compté...

EUREKA !

Nous allons utiliser le même code qui tournait sur le drone pour vous le présenter sous un nouvel angle : avec un simulateur.

## Desciption technique

Le PID calcule en temps réel la correction à appliquer aux moteurs pour annuler l'écart entre l'angle cible (0°) et l'angle mesuré. Il intègre un anti-windup (intégrale bornée à ±10), un clamp dérivée (±500°/s) et un reset à chaque armement. Les gains Kp=0.8, Kd=0.25 ont été déterminés empiriquement sur le vrai drone.
 
Le moteur physique est calibré sur les caractéristiques réelles du drone (masse 444g, frame 250mm, hover mesuré à 20%). Il inclut un filtre passe-bas (MOTOR_TAU=0.07s) qui modélise le délai ESC/moteur — c'est ce qui rend les oscillations du scénario 3 visibles, comme sur le vrai drone.

---
 
## Usage de l'Intelligence Artificielle
 
Nous avons utilisé l'IA uniquement comme **outil de compréhension et de documentation** : pour clarifier des concepts (filtre passe-bas, anti-windup, intégration d'Euler). L'IA n'a généré aucune fonction, aucune classe, aucun algorithme. Quand elle expliquait un concept, nous devions être capables de le réexpliquer sans elle avant de coder. **Le code est entièrement produit par nos soins.**
 
---
 
## Limites et évolutions
 
Le modèle actuel ne simule pas le yaw, ni les pertes moteur. Nous réparons actuellement le drone pour atteindre l'objectif initial : un drone autonome qui suit son utilisateur. Le code PID du simulateur est directement réutilisable sur le Pico RP2040 — c'est le même algorithme.
 
---
 
## Bibliothèques utilisées
 
- `pygame 2.6.1` — rendu graphique et interface utilisateur
- `numpy 2.4.2` — calculs vectoriels dans le moteur physique
 
Aucune bibliothèque de simulation de drone. Moteur physique et PID développés intégralement par l'équipe.