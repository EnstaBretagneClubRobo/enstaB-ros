# enstaB-ros
Ce repo contient tous les nodes qui ont été créé pour faire fonctionner un robot terrestre 
pour eurathlon 2015. Le but était de faire du mapping d'interieur en autonome, de bien nombreux nodes ont
été créés tous ne sont pas fonctionnels et beaucoup ont été écrit dans la précipitation et ne sont pas forcement bien documentés,
donc si vous avez des nodes qui font la même chose qu'un node présent et plus présentable autant le remplacer.
Mais ils peuvent servir d'exemples pour d'autres nodes.

### ai_mapping_robot
Ce node est une machine à état qui attend lance les nodes nécessaires au fonctionnement du robot (mais il 
y a des problèmes de compatibilité avec l'hokuyo si il n'est pas branché correctement.
Les nodes sont lancé par start_node.

### astar_path
Algorithme A* utilisant une occupancy grid (qui peut venir d'un hokuyo)

### autonmous_move_handling

Node devant prendre en compte la sortit du node A* mais n'a pas été testé.

### car_calibration
Non testé doit aider à calibrer les commandes du robot, ou aider à faire de l'inertielle.

### colour_detection
detecter des formes de certaines couleurs

### diagnostic_ros_system
Après avoir recut quel node surveiller, il vérifié qu'ils sont toujours présent et les relance si nécessaire.

### drift_detection
Compare différente méthode de localisation pour determiner si une diverge (ex slam hokuyo, slam kinect, inertiel).
Peut surement être amélioré.

### flux_video
Récupère un message image pour le retransmetre avec un framerate différent. 
C'était pour communiquer entre véhicule et station controle mais faire attention le faire de cette facon est gourmand en bande passante , surtout pendant un concours avec un wifi brouillé.


### gcs_eurathlon

Simple IHM pour controler le robot qui se sert la machine à état.

### gps_follow_car
Node de controle pour un robot autonome non fini (problème suivit de ligne).

### gps_handler
Service pour savoir si on est arrivé près du point voulu (mais pas forcement utile en tant que service)

### imgfile_to_pub

Node pour tester colour_dectection node.

### movie_save
Enregistre les images recues dans une vidéo.

### proxy_eura_smach
Node aidant à surveiller l'activité des nodes.

### pwm_send
Interface pour une carte polulu ou SSC-32U d'envoi de PWM.

### rc_receive
En utilisant mavros (mavlink for ros) permet de surveiller les commandes d'un controller rc connecté à une ardupilot.

### ruby 
Aide à la création d'occupancy grid pour l'utilisation d'A*.

### sauve_gps
Sauve les positions du robots directement dans un fichier kml lisible par google earth.

### save_2d_map
sauvegarde une occupancy grid sous forme binaire pour pouvoir la rejouer plus tard. (Il sauvegarde plusieurs au cause des risques de crash et de divergence.

### slider_sender
A utiliser avec colour_dectection node pour régler la détection de couleur. 

### start_node
Node peremettant de recevoir des msgs qui demande l'activation où désactivation de certains nodes.

### writeLaser 
Sauvegarde les données brutes de l'hokuyo
