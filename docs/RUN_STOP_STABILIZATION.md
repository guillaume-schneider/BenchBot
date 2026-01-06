# Stabilisation du Cycle RUN / STOP dans BenchBot

Ce document détaille les mesures techniques mises en œuvre pour garantir que le robot peut être arrêté et redémarré de manière fiable via l'interface graphique (GUI), sans laisser de processus "fantômes" ou rencontrer des blocages de navigation.

## 1. Stabilisation de Nav2 (Bonds & Lifecycle)

### Problème
Par défaut, les serveurs Nav2 utilisent un système de "Bonds" (battements de cœur) pour s'assurer que les nodes critiques sont toujours en vie. Lors d'un redémarrage rapide ou si le processeur est chargé, Nav2 peut échouer à établir ces liens, ce qui bloque le robot dans un état `INACTIVE`.

### Solutions implémentées
- **Désactivation des Bonds (`use_bond: false`)** : Nous avons configuré tous les serveurs Nav2 (`controller`, `planner`, `behavior`, `bt_navigator`, `map_server`, `smoother`, `waypoint_follower`) pour ne pas utiliser les bonds. Cela rend le démarrage beaucoup plus déterministe et robuste.
- **Extension des Timeouts** : Le `bond_timeout` du `lifecycle_manager` a été augmenté à **8.0 secondes** pour laisser plus de marge de manœuvre au système lors de l'activation des serveurs.

## 2. Fiabilité de l'Exploration (Explore Lite)

### Problème
L'explorateur est souvent le premier node à échouer s'il ne trouve pas immédiatement la carte ou le serveur d'action de Nav2. S'il démarre trop tôt, il s'arrête prématurément.

### Solutions implémentées
- **Délai de démarrage (`delay_s: 6.0`)** : Dans les matrices de test, nous avons augmenté le délai de lancement de l'explorateur à 6 secondes. Cela garantit que Nav2 a terminé sa phase de bringup avant que l'explorateur ne tente de commander le robot.
- **Correction des QoS** : Suppression de la confusion entre "Durability" et "Reliability". L'explorateur utilise maintenant correctement `transient_local` pour la durabilité de la carte, s'alignant sur les spécifications de SLAM Toolbox.

## 3. Gestion Proactive des Processus (Nettoyage "Nucléaire")

### Problème
Les processus ROS 2 et Gazebo créent souvent leurs propres groupes de processus (PGID). Un simple `SIGTERM` sur le parent ne suffit pas à arrêter les simulateurs lourds, laissant des instances "fantômes" qui occupent les ports réseau et les noms de nodes.

### Solutions implémentées
- **Nettoyage exhaustif (pkill)** : Le bouton **STOP** du GUI et l'orchestrateur utilisent maintenant une liste exhaustive de processus cibles à tuer via `pkill -9`. Cela inclut tous les serveurs individuels de Nav2 et Gazebo.
- **Purge du Démon ROS 2 (`ros2 daemon stop`)** : Il s'agit de la correction la plus critique. Nous arrêtons le démon de découverte ROS 2 entre chaque run pour vider le cache DDS. Cela évite que les nouveaux nodes ne tentent de se connecter à des nodes fantômes de la run précédente.
- **Délai de Cooldown (3.0s)** : Un délai de sécurité a été ajouté après le nettoyage pour s'assurer que les sockets réseau et la mémoire partagée du système d'exploitation sont totalement libérés avant d'instancier de nouveaux processus.

## 4. Alignement des Frames (Robot Base)

### Problème
Des erreurs de navigation survenaient car certains composants cherchaient `base_link` alors que le modèle TurtleBot3 utilise `base_footprint`.

### Solutions implémentées
- **Généralisation de `base_footprint`** : Toutes les configurations (`nav2_params.yaml`, `slam_toolbox_sync.yaml`, `explore_params.yaml`) ont été harmonisées pour utiliser `base_footprint`.

## Résumé du Workflow de Nettoyage
1. **Action utilisateur** : Clic sur STOP.
2. **Phase 1** : Envoi de `SIGKILL` au processus parent du worker BenchBot.
3. **Phase 2** : Arrêt du démon ROS 2 (`ros2 daemon stop`).
4. **Phase 3** : Exécution de `pkill -9` sur une liste de 20 processus critiques (Gazebo, Nav2, SLAM).
5. **Phase 4** : Attente de 3 secondes avant de libérer le verrou pour une nouvelle run.
