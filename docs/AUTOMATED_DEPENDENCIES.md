# Gestion Automatisée des Dépendances

L'Orchestrateur permet désormais de définir des dépendances externes (dépôts Git, workspaces à compiler) directement dans les fichiers de configuration des SLAMs ou des Datasets.

## Configuration

Ajoutez une section `dependencies` dans votre profil YAML (`configs/slams/*.yaml` ou `configs/datasets/*.yaml`) :

```yaml
dependencies:
  - name: "mon_slam_custom"
    git: "https://github.com/utilisateur/mon_slam.git" # URL Git à cloner
    branch: "main"                                   # Branche (optionnel)
    path: "deps/mon_slam_ws"                        # Chemin local de destination
    build: "colcon build --symlink-install"         # Commande de compilation
    source: "install/setup.bash"                    # Fichier à sourcer avant l'exécution
```

## Fonctionnement

1. **Clonage automatique** : Si le dossier spécifié dans `path` n'existe pas, l'orchestrateur clone automatiquement le dépôt Git.
2. **Compilation intelligente** : Si une commande `build` est fournie, elle est exécutée. Si le fichier spécifié dans `source` existe déjà, l'étape de compilation est ignorée pour gagner du temps.
3. **Sourcing dynamique** : Avant de lancer les processus (SLAM, scénario, etc.), l'orchestrateur source automatiquement tous les fichiers `setup.bash` définis dans les dépendances. Cela permet à ROS 2 de trouver vos packages personnalisés sans configuration manuelle de votre part.

## Exemple concret : GMapping

Le profil `configs/slams/gmapping.yaml` utilise maintenant ce système :
- Il clone `slam_gmapping` dans `deps/gmapping_ws`.
- Il le compile avec `colcon build`.
- Il lance le noeud via `ros2 run slam_gmapping slam_gmapping` sans script wrapper.
