# Guide de Référence Rapide: Exploration

## 🚀 Démarrage Rapide

### Lancer une Exploration

```bash
# Depuis le GUI
python3 gui/main.py
# Sélectionner test_slam_toolbox.yaml
# Cliquer "Run"

# Depuis la ligne de commande
python3 -m runner.run_matrix configs/matrices/test_slam_toolbox.yaml
```

### Vérifier que Ça Fonctionne

```bash
# 1. SLAM publie la carte
ros2 topic hz /map  # Devrait montrer ~1 Hz

# 2. Robot bouge
ros2 topic echo /odom --once  # Répéter, position change

# 3. Explorer trouve des frontières
ros2 topic echo /explore/frontiers --once
```

---

## 🔧 Configuration Minimale

### Explorer Parameters
```yaml
# configs/params/explore_params.yaml
explore_node:
  ros__parameters:
    robot_base_frame: base_footprint
    costmap_topic: /map
    min_frontier_size: 0.2
    transform_tolerance: 30.0
    use_sim_time: true
```

### Matrix Configuration
```yaml
# configs/matrices/test_slam_toolbox.yaml
defaults:
  run:
    warmup_s: 3.0
    timeout_s: 180.0
  probes:
    required:
      - type: topic_publish
        topic: /map
        timeout_s: 60

datasets:
  - scenario:
      processes:
        - name: nav2_sim
          cmd: [ros2, launch, ..., use_gazebo:=False]
        - name: explore
          delay_s: 2.0
          cmd: [ros2, launch, .../explore_with_qos.launch.py]
```

---

## 🐛 Problèmes Courants

### Explorer Ne Démarre Pas

**Symptôme**: "Waiting for costmap to become available"

**Solutions**:
1. Vérifier que SLAM publie: `ros2 topic list | grep map`
2. Vérifier le QoS: `ros2 topic info /map -v`
3. Vérifier les logs: `tail -f results/runs/LATEST/logs/explore.log`

### Robot Ne Bouge Pas

**Symptôme**: Position ne change pas

**Solutions**:
1. Vérifier Nav2: `ros2 node list | grep nav`
2. Vérifier cmd_vel: `ros2 topic echo /cmd_vel`
3. Vérifier les frontières: `ros2 topic echo /explore/frontiers`

### Gazebo Crash

**Symptôme**: "process has died [exit code -6/-9]"

**Solution**: Ajouter `use_gazebo:=False` dans la commande Nav2

### Coverage Faible

**Symptôme**: Coverage < 20%

**Solutions**:
1. Augmenter `timeout_s` (ex: 300s)
2. Réduire `min_frontier_size` (ex: 0.15)
3. Vérifier que l'explorer reçoit `/map` (QoS!)

---

## 📊 Métriques Attendues

| Métrique | Bon | Moyen | Mauvais |
|----------|-----|-------|---------|
| **Accessible Coverage** | >70% | 50-70% | <50% |
| **ATE RMSE** | <0.05m | 0.05-0.1m | >0.1m |
| **Path Length** | Variable | Variable | 0m |
| **Trajectoire Points** | >1000 | 500-1000 | <500 |

---

## 🔍 Commandes de Diagnostic

```bash
# Lister les nœuds actifs
ros2 node list

# Vérifier un topic
ros2 topic info /map -v
ros2 topic hz /map
ros2 topic echo /map --once

# Vérifier les TF
ros2 run tf2_ros tf2_echo map base_footprint

# Tuer Gazebo si bloqué
pkill -9 gzserver gzclient

# Voir les logs en temps réel
tail -f results/runs/LATEST/logs/*.log
```

---

## 📁 Fichiers Importants

```
benchbot/
├── configs/
│   ├── matrices/
│   │   └── test_slam_toolbox.yaml      # Configuration principale
│   └── params/
│       └── explore_params.yaml         # Paramètres explorer
├── tools/
│   └── launch/
│       └── explore_with_qos.launch.py  # Launch avec QoS fix
├── docs/
│   ├── EXPLORATION_FIX_COMPLETE.md     # Doc complète
│   ├── ROBUST_SYNCHRONIZATION.md       # Guide probes
│   └── TROUBLESHOOTING_EXPLORATION.md  # Troubleshooting
└── runner/
    └── orchestrator.py                 # Logique d'exécution
```

---

## ⚡ Tips & Tricks

### Accélérer les Tests
```yaml
defaults:
  run:
    timeout_s: 60  # Au lieu de 180
    warmup_s: 1.0  # Au lieu de 3
```

### Déboguer en Détail
```yaml
# Ajouter dans la commande explore
- --ros-args
- --log-level
- debug
```

### Visualiser en Temps Réel
```bash
# Lancer RViz séparément
ros2 launch nav2_bringup rviz_launch.py
```

---

## 🎯 Checklist Avant Run

- [ ] Tuer processus Gazebo précédents: `pkill -9 gzserver gzclient`
- [ ] Vérifier config: `use_gazebo:=False`
- [ ] Vérifier probes configurées
- [ ] Vérifier `delay_s` pour explorer
- [ ] Vérifier espace disque suffisant

---

## 📞 Support

**Documentation**: `docs/EXPLORATION_FIX_COMPLETE.md`  
**Troubleshooting**: `docs/TROUBLESHOOTING_EXPLORATION.md`  
**Probes**: `docs/ROBUST_SYNCHRONIZATION.md`

---

**Dernière mise à jour**: 2026-01-06  
**Version**: 1.0
