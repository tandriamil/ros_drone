# Pikopter
Projet de Master 1 consistant à la réalisation d'un module de commande de drône
via scripts avec l'aide du framework [ros](http://www.ros.org).

## Utilisation Pikopter
Lancer le simulateur via la commande `-> sim_vehicle.sh -v ArduCopter --out
10.5.5.1:5760 --console --map`. Bien attendre que la console du simulateur
affiche le message `GPS lock at 0 meters` avant de passer à l'étape suivante.

Sur le Raspberry Pi lancer le fichier `.launch` qui exécute tous les noeuds
```shell
roslaunch pikopter simul.launch client_ip:=`echo $SSH_CLIENT | awk '{print $1}'`
```
Bien attendre que tous les noeuds soient bien tous lancés avant de procéder à
l'étape suivante.

Lancer lua dans le repertoire `DEBUG` de Jakopter via la commande `lua`. Il faut
pour le moment entrer les commandes à la main et non dans un script lua. Exemple
de suite de commandes pour tester le fonctionnement du drone (simulé) :
```lua
> l=require("libjakopter")
> l.connect("10.5.5.1")
> l.takeoff()
> l.forward(1.0)
> l.down(1.0)
> l.land()
```

La liste des commandes qui fonctionnent pour le moment :
- takeoff()
- land()
- up(float speed)
- down(float speed)
- forward(float speed)
- backward(float speed)

Notons que `speed` doit être compris entre `-1.0` et `1.0`.
