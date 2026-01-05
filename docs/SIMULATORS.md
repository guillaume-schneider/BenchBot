# Simulator Management

Le SLAM Bench Orchestrator supporte maintenant plusieurs simulateurs : **Gazebo** et **O3DE (Open 3D Engine)**.

## 🎮 Simulators Supportés

### Gazebo Classic
- **Installation** : Package système (`ros-humble-gazebo-ros-pkgs`)
- **Taille** : ~500 MB
- **Niveau de support** : ✅ Production-ready
- **Avantages** : Bien documenté, large communauté ROS
- **Limitations** : Physique moins moderne, parfois instable

### O3DE (Open 3D Engine)
- **Installation** : Automatique via l'interface GUI
- **Taille** : ~15 GB (avec dépendances)
- **Niveau de support** : 🚧 Expérimental
- **Avantages** : Physique PhysX moderne, meilleurs graphismes, plus stable
- **Limitations** : Setup initial long (~30-60 min), moins de documentation ROS 2

## 📦 Installation

### Via GUI (Recommandé)

1. Ouvrez l'orchestrateur : `python3 gui/main.py`
2. Allez dans l'onglet **Tools → Simulators**
3. Cliquez sur **"Install O3DE"**
4. Attendez la fin de l'installation (peut prendre 30-60 min)

### Manual Installation

```bash
# Installer O3DE manuellement
python3 -c "from tools.simulator_manager import SimulatorManager; mgr = SimulatorManager(); mgr.ensure_installed('o3de')"
```

## 🗺️ Conversion SDF → O3DE

L'orchestrateur convertit **automatiquement** vos mondes Gazebo (SDF) en projets O3DE !

### Format SDF Supporté

```xml
<sdf version='1.6'>
  <world name='my_world'>
    <model name='wall'>
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box><size>2 0.1 2</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Géométries Supportées
- ✅ `<box>` → BoxShapeComponent
- ✅ `<cylinder>` → CylinderShapeComponent  
- ✅ `<sphere>` → SphereShapeComponent
- ⚠️ `<mesh>` → Partiellement supporté

## 🚀 Utilisation

### Option 1 : Dataset O3DE Dédié

Créez `configs/datasets/my_dataset_o3de.yaml` :

```yaml
name: "My Dataset with O3DE"
id: "my_o3de_dataset"

simulator: "o3de"  # <-- Force O3DE

scenario:
  world_model: "worlds/model.sdf"  # Même format que Gazebo!
  # ... reste de la config
```

### Option 2 : Switch Dynamique

Vous pouvez aussi dupliquer votre config Gazebo et juste changer `simulator: gazebo` → `simulator: o3de`.

### Lancement

```bash
# Via GUI
python3 gui/main.py
# Sélectionnez votre config O3DE et lancez normalement

# Via CLI
python3 runner/run_one.py configs/matrices/my_o3de_matrix.yaml
```

## 🔍 Vérification d'Installation

```python
from tools.simulator_manager import SimulatorManager

mgr = SimulatorManager()
info = mgr.list_simulators()

for sim_name, sim_info in info.items():
    print(f"{sim_name}:")
    print(f"  Installed: {sim_info['installed']}")
    print(f"  Version: {sim_info['version']}")
    print(f"  Dependencies: {sim_info['dependencies']}")
```

## 🧪 Benchmark Comparatif Gazebo vs O3DE

Pour comparer les performances d'un algo SLAM sur les deux simulateurs :

```yaml
# configs/matrices/compare_sim.yaml
name: "Gazebo vs O3DE Comparison"

datasets:
  - include: "configs/datasets/tb3_sim_explore_modeA.yaml"  # Gazebo
  - include: "configs/datasets/tb3_o3de_explore.yaml"       # O3DE

slams:
  - include: "configs/slams/slam_toolbox.yaml"

matrix:
  include:
    - datasets: ["tb3_sim_explore_modeA", "tb3_o3de_explore"]
      slams: ["slam_toolbox"]
      seeds: [0]
      repeats: 3
```

Lancez et comparez les métriques (Coverage, IoU, ATE) entre Gazebo et O3DE !

## ⚙️ Architecture

```
tools/
├── simulator_manager.py       # Gestionnaire central
└── simulators/
    ├── __init__.py
    ├── base.py               # Interface abstraite
    ├── gazebo.py             # Implémentation Gazebo
    └── o3de.py               # Implémentation O3DE
                              #   - Installation automatique
                              #   - Conversion SDF → O3DE
                              #   - Gestion projet O3DE
```

## 🐛 Debugging

### O3DE ne démarre pas

```bash
# Vérifier les dépendances
python3 -c "from tools.simulators.o3de import O3DESimulator; sim = O3DESimulator(); print(sim.verify_dependencies())"

# Cleanup forcé
pkill -9 -f o3de
pkill -9 -f AssetProcessor
```

### Conversion SDF échoue

Vérifiez que votre SDF utilise des géométries supportées (box, cylinder, sphere). Les meshes complexes peuvent nécessiter un traitement manuel.

## 📚 Ressources

- [O3DE ROS 2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2)
- [O3DE Documentation](https://www.o3de.org/docs/)
- [Gazebo Classic](http://gazebosim.org/)

## 🎯 Roadmap

- [ ] Support meshes complexes
- [ ] Import direct de worlds Ignition Gazebo
- [ ] Template de robot TurtleBot3 optimisé pour O3DE
- [ ] Paramètres physiques configurables
- [ ] Support sensors (depth camera, etc.)
