#########################################################
# 				  Utilisation Pikopter					#
#########################################################

1) Lancer le simulateur
	-> sim_vehicle.sh -v ArduCopter --out 10.5.5.1:5760 --console --map
	NB : Bien attendre côté console du simulateur le message "GPS lock at 0 meters" avant de faire l'étape 2)

2) Sur le Raspberry Pi lancer le fichier .launch qui lance tous les noeuds
	-> roslaunch pikopter simul.launch client_ip:=`echo $SSH_CLIENT | awk '{print $1}'`
	NB : Bien attendre que tous les noeuds soient bien tous lancés avant de procéder à l'étape 3)

3) Lancer lua dans le repertoire DEBUG de Jakopter
	-> lua
	NB : Il faut pour le moment entrer les commandes à la main et pas dans un script lua
	Exemple de suite de commandes pour tester le fonctionnement du drone (simulé) :
		> l=require("libjakopter")
		> l.connect("10.5.5.1")
		> l.takeoff()
		> l.forward(1.0)
		> l.down(1.0)
		> l.land()

	La liste des commandes qui fonctionnent pour le moment :
		- takeoff()
		- land()
		- up(float speed)
		- down(float speed)
		- forward(float speed)
		- backward(float speed)
		
		speed doit être compris entre -1.0 et 1.0
