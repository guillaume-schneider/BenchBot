# Correction de test_cartographer.yaml

## Problème identifié

Le run `2026-01-05_00-07-30` utilisait la matrice `test_cartographer.yaml` qui avait une **configuration obsolète** pour explore_lite :

### Configuration obsolète (avant)
```yaml
- name: explore
  cmd:
  - /home/schneigu/Projects/slam_bench_orchestrator/tools/launch/explore_wrapper.sh
  - ros2
  - run          # ← Problème : ros2 run n'applique pas les paramètres inline
  - explore_lite
  - explore
  params:
    explore_node:
      ros__parameters:
        robot_base_frame: base_footprint
        min_frontier_size: 0.15
        planner_frequency: 1.0
        use_sim_time: true
        # ← Manque: costmap_topic: map
        # ← Manque: transform_tolerance: 5.0
```

### Conséquences
- explore_lite cherchait `costmap` au lieu de `map`
- Timeout TF trop court (0.3s par défaut)
- Le robot ne se déplaçait pas
- **Total Path Length: 0.01 m** (seulement 1 cm)
- **Map Coverage: 6.61%** (très faible)

## Correction appliquée

### Configuration corrigée (après)
```yaml
- name: explore
  cmd:
  - ros2
  - launch       # ← Changé : ros2 launch applique les paramètres
  - explore_lite
  - explore.launch.py
  - use_sim_time:=True
  env:
    AMENT_PREFIX_PATH: .../explore_lite:/opt/ros/humble
    LD_LIBRARY_PATH: ...
    PYTHONPATH: ...
  params:
    explore_node:
      ros__parameters:
        robot_base_frame: base_footprint
        costmap_topic: map              # ← Ajouté
        min_frontier_size: 0.15
        planner_frequency: 1.0
        transform_tolerance: 5.0        # ← Ajouté
        use_sim_time: true
```

## Différence ros2 run vs ros2 launch

### `ros2 run explore_lite explore`
- Lance directement l'exécutable
- **N'applique PAS les paramètres inline** de la section `params`
- Utilise uniquement le fichier de config par défaut
- ❌ Ne fonctionne pas avec notre configuration

### `ros2 launch explore_lite explore.launch.py`
- Lance via un fichier launch
- **Applique les paramètres inline** correctement
- Peut surcharger les paramètres du fichier de config
- ✅ Fonctionne avec notre configuration

## Fichiers modifiés

1. ✅ `/home/schneigu/Projects/slam_bench_orchestrator/configs/matrices/test_cartographer.yaml`
   - Changé `ros2 run` → `ros2 launch`
   - Ajouté `costmap_topic: map`
   - Ajouté `transform_tolerance: 5.0`
   - Ajouté les variables d'environnement nécessaires

## Test à relancer

Pour tester avec la configuration corrigée :

```bash
cd /home/schneigu/Projects/slam_bench_orchestrator
python3 tools/run_batch.py configs/matrices/test_cartographer.yaml
```

Ou depuis le GUI, relancer un test avec la matrice `test_cartographer.yaml`.

## Résultats attendus

Avec la configuration corrigée, vous devriez voir :
- ✅ explore_lite se connecte au serveur Nav2
- ✅ Le robot se déplace et explore
- ✅ Total Path Length > 10 m
- ✅ Map Coverage > 50%
- ✅ RMSE ~0.70 m (comme le test de 23:57)

## Note importante

Il y a maintenant **deux matrices de test** :

1. **`slam_comparison.yaml`** - Compare plusieurs algorithmes SLAM
   - NoOp, SLAM Toolbox, Cartographer
   - Configuration correcte ✅

2. **`test_cartographer.yaml`** - Test uniquement Cartographer
   - Configuration maintenant corrigée ✅

Les deux utilisent maintenant la même configuration pour explore_lite.
