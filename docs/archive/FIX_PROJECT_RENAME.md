# Résolution des Problèmes liés au Renommage du Projet

**Date:** 05 Janvier 2026
**Sujet:** Correction des dépendances et configurations suite au renommage de `slam_bench_orchestrator` vers `benchbot`.

## 1. Description du Problème

Suite au renommage du répertoire principal du projet de `slam_bench_orchestrator` vers `benchbot`, l'orchestrateur ne parvenait plus à lancer correctement les simulations.

### Symptômes
1.  **Exploration bloquée** : Pas d'interaction ROS2 visible, le robot ne bougeait pas.
2.  **Erreur de package manquants** : `Package 'explore_lite' not found`.
3.  **Logs d'erreurs** : Les logs montraient que le système cherchait des paquets dans `/opt/ros/humble` au lieu de l'environnement local.
4.  **Chemins incorrects** : Les variables d'environnement (`AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`) pointaient vers l'ancien dossier `/home/schneigu/Projects/slam_bench_orchestrator/...`.

## 2. Analyse des Causes

1.  **Artefacts de Build Persistants** : Les dossiers de build (`deps/install`, `build`, etc.) contenaient des fichiers de configuration (comme `setup.bash`) générés avec les *chemins absolus* de l'ancien nom de dossier. ROS 2 / Colcon ne gère pas automatiquement le déplacement de dossiers de build.
2.  **Configurations Hardcodées** : De nombreux fichiers YAML (`configs/matrices/*.yaml`, `configs/datasets/*.yaml`) et scripts de lancement (`tools/launch/*.sh`) contenaient encore la chaîne de caractères `slam_bench_orchestrator`.
3.  **Définition de Dataset Incomplète** : Le fichier de matrice utilisé (`copy_default.yaml`) définissait le dataset `tb3_sim_explore_modeA` inline mais omettait la section `dependencies`. Conséquence : après nettoyage des builds, l'orchestrateur ne savait pas qu'il devait reconstruire `m-explore`.

## 3. Actions Correctives

### 3.1. Nettoyage Complet
Suppression de tous les artefacts générés qui pouvaient contenir des chemins obsolètes :
```bash
rm -rf deps/install deps/log deps/*/build deps/*/install deps/*/log
find . -name "__pycache__" -type d -exec rm -rf {} +
```

### 3.2. Mise à Jour des Configurations
Remplacement massif de l'ancien nom de projet par le nouveau dans tous les fichiers de configuration et les scripts :
```bash
sed -i 's/slam_bench_orchestrator/benchbot/g' configs/matrices/*.yaml configs/datasets/*.yaml configs/slams/*.yaml tools/launch/*.sh configs/matrix.yaml CONTRIBUTING.md README.md
```

### 3.3. Correction des Définitions de Dépendances
Ajout explicite de la section `dependencies` dans `configs/matrices/copy_default.yaml` et `configs/matrix.yaml` pour garantir que `m-explore` soit reconstruit au lancement :

```yaml
datasets:
  - id: tb3_sim_explore_modeA
    kind: sim_gazebo
    dependencies:  # SECTION AJOUTÉE
    - name: m-explore
      git: https://github.com/robo-friends/m-explore-ros2
      branch: main
      path: deps/m_explore_ws
      build: colcon build --symlink-install
      source: install/setup.bash
    scenario: ...
```

### 3.4. Correction des Chemins d'Environnement
Mise à jour des chemins dans la section `env` des configurations pour pointer vers le dossier de build local spécifique (`deps/m_explore_ws/install`) au lieu de l'ancien dossier générique `deps/install` :

```yaml
env:
  AMENT_PREFIX_PATH: /home/schneigu/Projects/benchbot/deps/m_explore_ws/install/multirobot_map_merge:...
  # ... (LD_LIBRARY_PATH, PYTHONPATH mis à jour de la même façon)
```

## 4. Résultat

L'orchestrateur :
1.  Détecte correctement la dépendance manquante.
2.  Clone et compile `m-explore` dans le bon dossier.
3.  Source les bons fichiers d'environnement.
4.  Lance `explore_lite` avec succès, permettant l'exploration autonome du robot.
