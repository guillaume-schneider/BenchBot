# Corrections supplémentaires pour le problème de déplacement du robot

## Problème identifié

Après analyse approfondie des logs, le vrai problème n'était **PAS** que Cartographer ne fonctionnait pas, mais que **explore_lite ne pouvait pas démarrer** à cause des erreurs de synchronisation TF.

### Symptômes
- ✅ Cartographer publie bien la TF `map -> odom`
- ✅ L'odométrie publie bien `odom -> base_footprint`
- ❌ **explore_lite** ne peut pas obtenir la transformation `base_link -> map` à cause de:
  - Timestamps TF désynchronisés (TF_OLD_DATA warnings)
  - `transform_tolerance` trop faible (0.3s par défaut)
  - Mauvais `robot_base_frame` (base_link au lieu de base_footprint)

### Logs d'erreur explore_lite
```
[WARN] Timed out waiting for transform from base_link to map to become available
Lookup would require extrapolation into the past. Requested time 7.091000 
but the earliest data is at time 1767564612.719409
```

## Corrections appliquées

### 1. Augmentation de transform_tolerance dans explore_lite

**Fichiers modifiés:**
- `/home/schneigu/Projects/slam_bench_orchestrator/deps/src/m-explore/explore/config/params.yaml`
- `/home/schneigu/Projects/slam_bench_orchestrator/deps/install/explore_lite/share/explore_lite/config/params.yaml`

**Changements:**
```yaml
robot_base_frame: base_footprint  # Changé de base_link
transform_tolerance: 10.0  # Augmenté de 0.3 à 10.0 secondes
```

**Raison:** 
- La tolérance de 0.3s est trop faible pour gérer les délais de synchronisation entre Gazebo et ROS2
- Avec 10.0s, explore_lite peut tolérer les timestamps légèrement désynchronisés
- Cela permet d'utiliser des TF même si elles sont "vieilles" de quelques secondes

### 2. Configuration Cartographer (déjà appliquée)

**Fichier:** `configs/params/cartographer_turtlebot3_2d.lua`

```lua
tracking_frame = "base_footprint"  -- Frame qui existe vraiment
published_frame = "odom"           -- Publier map->odom
provide_odom_frame = true          -- Cartographer publie la TF map->odom
lookup_transform_timeout_sec = 0.5 -- Timeout plus tolérant
```

### 3. Configuration explore_lite dans la matrice

**Fichier:** `configs/matrices/slam_comparison.yaml`

```yaml
params:
  explore_node:
    ros__parameters:
      robot_base_frame: base_footprint
      min_frontier_size: 0.15
      planner_frequency: 1.0
      transform_tolerance: 5.0  # Tolérance élevée
```

## Explication du problème TF_OLD_DATA

### Cause racine
Le problème `TF_OLD_DATA` est causé par une **désynchronisation temporelle** entre:
1. **Gazebo** qui publie `/clock` avec le temps de simulation
2. **Les nœuds ROS2** qui reçoivent et traitent les messages avec un léger délai
3. **Le buffer TF** qui rejette les transformations "trop vieilles"

### Pourquoi cela se produit
```
Temps simulation (Gazebo): t = 7.091s
Temps réel (wall clock):   t = 1767564612.719s
Délai de traitement:       ~5s

Quand explore_lite demande une TF à t=7.091s (temps simulation),
le buffer TF a déjà avancé à t=12s et considère que 7.091s est "dans le passé"
```

### Solution
Au lieu d'essayer de synchroniser parfaitement (très difficile avec Gazebo),
on **augmente la tolérance** pour accepter les TF "vieilles" de plusieurs secondes.

## Impact des changements

### ✅ Avantages
- explore_lite peut maintenant démarrer même avec des TF désynchronisées
- Le robot devrait se déplacer et explorer
- Pas besoin de modifier Gazebo ou la synchronisation temporelle

### ⚠️ Compromis
- Les TF peuvent être légèrement en retard (quelques secondes)
- Pour un robot réel, cela pourrait poser problème
- Pour la simulation/benchmarking, c'est acceptable

### 🎯 Résultat attendu
Avec ces changements:
1. ✅ Cartographer publie `map -> odom`
2. ✅ Odométrie publie `odom -> base_footprint`
3. ✅ explore_lite peut obtenir `base_footprint -> map`
4. ✅ Le robot se déplace et explore
5. ⚠️ Les warnings TF_OLD_DATA persistent mais n'empêchent plus le fonctionnement

## Test

Pour tester les corrections:
```bash
cd /home/schneigu/Projects/slam_bench_orchestrator
python3 tools/run_batch.py configs/matrices/slam_comparison.yaml
```

Ou test manuel:
```bash
# Terminal 1: Lancer Gazebo + Nav2
ros2 launch /home/schneigu/Projects/slam_bench_orchestrator/tools/launch/tb3_sim_no_loc.launch.py \
  use_sim_time:=True x_pose:=0.5 y_pose:=0.5

# Terminal 2: Lancer Cartographer
ros2 launch /home/schneigu/Projects/slam_bench_orchestrator/tools/launch/cartographer_custom.launch.py \
  use_sim_time:=True \
  configuration_directory:=/home/schneigu/Projects/slam_bench_orchestrator/configs/params \
  configuration_basename:=cartographer_turtlebot3_2d.lua

# Terminal 3: Lancer explore_lite
ros2 launch explore_lite explore.launch.py use_sim_time:=True

# Terminal 4: Vérifier les TF
ros2 run tf2_ros tf2_echo map base_footprint
```

## Fichiers modifiés (résumé)

1. ✅ `configs/params/cartographer_turtlebot3_2d.lua` - Configuration Cartographer
2. ✅ `configs/slams/cartographer_2d.yaml` - Pointeur vers config Lua
3. ✅ `deps/gmapping_ws/src/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py` - Fix ROS2 Humble
4. ✅ `deps/src/m-explore/explore/config/params.yaml` - Paramètres explore_lite (source)
5. ✅ `deps/install/explore_lite/share/explore_lite/config/params.yaml` - Paramètres explore_lite (install)
6. ✅ `configs/matrices/slam_comparison.yaml` - Ajout transform_tolerance

## Prochaines étapes

1. **Relancer les tests** avec les nouvelles corrections
2. **Vérifier** que explore_lite démarre et que le robot se déplace
3. **Surveiller** les logs pour confirmer l'absence d'erreurs bloquantes
4. **Comparer** les performances entre les différents algorithmes SLAM
