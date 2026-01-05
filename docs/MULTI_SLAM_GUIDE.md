# 🎯 Guide de Benchmarking Multi-SLAM

## 📋 SLAMs Disponibles

Votre orchestrateur supporte actuellement **4 algorithmes SLAM** :

### 1. **SLAM Toolbox (Sync Mode)**
- **ID** : `slam_toolbox_sync`
- **Type** : Graph-based SLAM
- **Avantages** : Excellent loop closure, robuste
- **Config** : `configs/slams/slam_toolbox_sync.yaml`

### 2. **Cartographer 2D**
- **ID** : `cartographer_2d`
- **Type** : Google's Cartographer
- **Avantages** : Très précis, bon sur grands environnements
- **Config** : `configs/slams/cartographer_2d.yaml`

### 3. **GMapping-like**
- **ID** : `gmapping_like`
- **Type** : Particle filter SLAM
- **Avantages** : Classique, rapide
- **Config** : `configs/slams/gmapping_like.yaml`

### 4. **NoOp (Baseline)**
- **ID** : `noop`
- **Type** : Pas de SLAM (odométrie uniquement)
- **Avantages** : Baseline pour comparaison
- **Config** : `configs/slams/noop.yaml`

---

## 🚀 Comment Tester Différents SLAMs

### Option 1 : Via GUI (Recommandé)

```bash
cd ~/Projects/slam_bench_orchestrator
python3 gui/main.py
```

1. Dans le **Dashboard**, sélectionnez **`slam_comparison.yaml`**
2. Cliquez **Run**
3. L'orchestrateur va lancer **3 benchmarks** automatiquement :
   - Run 1 : NoOp (baseline)
   - Run 2 : SLAM Toolbox
   - Run 3 : Cartographer
4. Les résultats s'affichent automatiquement !

### Option 2 : Via CLI

```bash
cd ~/Projects/slam_bench_orchestrator
# Source ROS 2
source /opt/ros/humble/setup.bash

# Lancer la matrice
python3 runner/run_matrix.py configs/matrices/slam_comparison.yaml
```

---

## 📊 Comparer les Résultats

### Métriques Collectées pour Chaque SLAM

- **Coverage** : % de la carte explorée
- **IoU** : Intersection over Union (précision de la carte)
- **ATE** : Absolute Trajectory Error (précision de localisation)
- **Map Quality** : Netteté de la carte générée
- **Runtime** : Temps total d'exécution
- **CPU/Memory** : Ressources utilisées

### Visualiser la Comparaison

Dans le GUI :
1. Allez dans **Details** (cliquez sur la card de votre matrice)
2. Onglet **Results** : Tableau comparatif
3. Cliquez sur chaque run pour voir les métriques détaillées
4. Comparez visuellement les cartes générées

---

## 🔧 Créer Votre Propre Matrice

### Exemple : Tester Seulement 2 SLAMs

```yaml
# configs/matrices/my_test.yaml
name: "My SLAM Test"

datasets:
  - include: "configs/datasets/tb3_sim_explore_modeA.yaml"

slams:
  - id: slam_toolbox_sync
    profile: configs/slams/slam_toolbox_sync.yaml
  - id: cartographer_2d
    profile: configs/slams/cartographer_2d.yaml

matrix:
  include:
    - dataset: tb3_sim_explore_modeA
      slams: [slam_toolbox_sync, cartographer_2d]
      seeds: [0]
      repeats: 1
```

### Exemple : Tester avec Plusieurs Seeds (Robustesse)

```yaml
matrix:
  include:
    - dataset: tb3_sim_explore_modeA
      slams: [slam_toolbox_sync]
      seeds: [0, 1, 2, 3, 4]  # 5 runs différents
      repeats: 1
```

Cela va générer **5 runs** avec différentes initialisations aléatoires.

---

## 📝 Résultats Attendus

### Structure des Résultats

```
results/runs/
├── 2026-01-04_XX-XX-XX__tb3_sim_explore_modeA__noop__seed0__r0/
│   ├── bags/       # Rosbag enregistré
│   ├── logs/       # Logs de chaque process
│   └── metrics.json # Métriques calculées
├── 2026-01-04_XX-XX-XX__tb3_sim_explore_modeA__slam_toolbox_sync__seed0__r0/
│   └── ...
└── 2026-01-04_XX-XX-XX__tb3_sim_explore_modeA__cartographer_2d__seed0__r0/
    └── ...
```

### Exemple de Comparaison

| SLAM | Coverage | IoU | ATE | Runtime |
|------|----------|-----|-----|---------|
| NoOp | 45% | 0.35 | 2.5m | 90s |
| SLAM Toolbox | **78%** | **0.82** | **0.3m** | 95s |
| Cartographer | 72% | 0.79 | 0.4m | 110s |

**Gagnant** : SLAM Toolbox (meilleure couverture et précision)

---

## 🐛 Troubleshooting

### Cartographer Ne Lance Pas

**Vérifiez** que Cartographer est installé :
```bash
ros2 pkg list | grep cartographer
```

**Si absent**, installez :
```bash
sudo apt install ros-humble-cartographer-ros
```

### GMapping Ne Fonctionne Pas

GMapping n'a pas de port officiel ROS 2. Utilisez :
- SLAM Toolbox (meilleur)
- Cartographer (alternative)

### Erreur "Failed to resolve dependencies"

Une config SLAM fait référence à un fichier manquant. **Vérifiez** :
```bash
cat configs/slams/cartographer_2d.yaml
# Regardez les chemins dans 'configuration_directory'
```

**Adaptez** les chemins à votre système.

---

## 🎯 Recommandations

### Pour Débuter
1. **Testez** d'abord avec `noop` (baseline)
2. **Puis** `slam_toolbox_sync` (le plus robuste)
3. **Comparez** avec votre objectif

### Pour Performance
- **SLAM Toolbox** : Meilleur équilibre vitesse/précision
- **Cartographer** : Plus précis sur grands environnements
- **NoOp** : Le plus rapide (pas de SLAM)

### Pour Recherche
- **Multi-seeds** : Tester robustesse
- **Multi-datasets** : Tester généralisation
- **Multi-slams** : Benchmarking comparatif

---

## 📚 Ressources

- **SLAM Toolbox** : https://github.com/SteveMacenski/slam_toolbox
- **Cartographer** : https://github.com/cartographer-project/cartographer
- **ROS 2 SLAM** : https://github.com/ros-planning/navigation2

---

## 🎉 Prochain Niveau

### Ajouter un Nouveau SLAM

1. Créez `configs/slams/my_slam.yaml`
2. Définissez la commande de lancement
3. Ajoutez-le à votre matrice
4. Lancez !

**Exemple** : Ajouter Hector SLAM :

```yaml
# configs/slams/hector_slam.yaml
schema_version: 1
id: "hector_slam"
display_name: "Hector SLAM"

launch:
  cmd: ["ros2", "launch", "hector_slam", "hector_slam.launch.py"]
  use_sim_time: true

io_contract:
  map_topic: "/map"
  scan_topic: "/scan"

probes:
  ready:
    - type: topic_publish
      topic: /map
      timeout_s: 60
```

Puis ajoutez-le à votre matrice !

---

**Happy Benchmarking !** 🚀
