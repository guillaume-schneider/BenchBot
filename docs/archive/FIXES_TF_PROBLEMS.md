# Corrections pour les problèmes TF et déplacement du robot

## Problèmes identifiés

### 1. Cartographer - Problèmes de TF
**Symptômes dans les logs :**
- `TF_OLD_DATA ignoring data from the past for frame odom`
- Le robot ne se déplace pas
- explore_lite ne peut pas fonctionner

**Causes :**
1. Configuration Lua incorrecte :
   - `tracking_frame = "imu_link"` → Le TurtleBot3 Waffle n'a pas ce frame
   - `provide_odom_frame = false` → Cartographer ne publie pas la TF `map -> odom`
   - `lookup_transform_timeout_sec = 0.2` → Timeout trop court

**Solution appliquée :**
✅ Créé `/home/schneigu/Projects/slam_bench_orchestrator/configs/params/cartographer_turtlebot3_2d.lua`
   - `tracking_frame = "base_footprint"` (frame qui existe sur le robot)
   - `provide_odom_frame = true` (pour publier map->odom)
   - `lookup_transform_timeout_sec = 0.5` (timeout plus tolérant)

✅ Modifié `/home/schneigu/Projects/slam_bench_orchestrator/configs/slams/cartographer_2d.yaml`
   - Pointe maintenant vers le nouveau fichier Lua personnalisé

### 2. GMapping - Problème de compatibilité ROS2
**Symptômes dans les logs :**
- Le nœud démarre et fonctionne partiellement
- Segmentation fault à la fin
- Problèmes de TF similaires

**Causes :**
1. Launch file utilise `node_executable` (syntaxe ROS2 Foxy) au lieu de `executable` (ROS2 Humble)
2. Pas de configuration explicite pour `use_sim_time`

**Solution appliquée :**
✅ Modifié `/home/schneigu/Projects/slam_bench_orchestrator/deps/gmapping_ws/src/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py`
   - Changé `node_executable` → `executable`

✅ Recompilé le package gmapping :
   ```bash
   cd /home/schneigu/Projects/slam_bench_orchestrator/deps/gmapping_ws
   colcon build --packages-select slam_gmapping
   ```

## Vérifications à faire

### Pour Cartographer :
1. Vérifier que le fichier Lua est bien chargé :
   ```bash
   # Dans les logs, vous devriez voir :
   # -configuration_directory /home/schneigu/Projects/slam_bench_orchestrator/configs/params
   # -configuration_basename cartographer_turtlebot3_2d.lua
   ```

2. Vérifier que la TF map->odom est publiée :
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   ```

3. Vérifier qu'il n'y a plus d'erreurs TF_OLD_DATA dans les logs

### Pour GMapping :
1. Vérifier que le nœud démarre correctement :
   ```bash
   # Le wrapper doit charger le bon workspace
   source /home/schneigu/Projects/slam_bench_orchestrator/deps/gmapping_ws/install/setup.bash
   ros2 run slam_gmapping slam_gmapping --ros-args -p use_sim_time:=true
   ```

2. Vérifier la publication de la TF :
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   ```

## Prochaines étapes

1. **Relancer les tests** avec les configurations corrigées
2. **Surveiller les logs** pour vérifier :
   - Absence d'erreurs TF_OLD_DATA
   - Publication correcte de map->odom
   - Déplacement effectif du robot (explore_lite doit fonctionner)
3. **Si le problème persiste**, vérifier :
   - Que tous les nœuds utilisent `use_sim_time:=true`
   - Que Gazebo publie bien `/clock`
   - Que les timestamps des messages sont cohérents

## Fichiers modifiés

1. ✅ `/home/schneigu/Projects/slam_bench_orchestrator/configs/params/cartographer_turtlebot3_2d.lua` (créé)
2. ✅ `/home/schneigu/Projects/slam_bench_orchestrator/configs/slams/cartographer_2d.yaml` (modifié)
3. ✅ `/home/schneigu/Projects/slam_bench_orchestrator/deps/gmapping_ws/src/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py` (modifié)
4. ✅ Package `slam_gmapping` recompilé

## Commande pour tester

```bash
# Relancer un test avec cartographer et gmapping
cd /home/schneigu/Projects/slam_bench_orchestrator
python3 tools/run_batch.py configs/matrices/slam_comparison.yaml

# Ou tester manuellement cartographer
ros2 launch /home/schneigu/Projects/slam_bench_orchestrator/tools/launch/cartographer_custom.launch.py \
  use_sim_time:=True \
  configuration_directory:=/home/schneigu/Projects/slam_bench_orchestrator/configs/params \
  configuration_basename:=cartographer_turtlebot3_2d.lua
```
