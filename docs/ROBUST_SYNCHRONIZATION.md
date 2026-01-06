# Synchronisation Robuste Sans Délais Fixes

## 🎯 Problème

Les délais fixes (`delay_s`, `warmup_s`) ne sont pas robustes:
- ❌ Trop courts → Le système n'est pas prêt, erreurs
- ❌ Trop longs → Temps perdu inutilement
- ❌ Dépendent de la machine (rapide vs lente)
- ❌ Pas déterministes

## ✅ Solution: Probes Actives

Au lieu d'attendre un temps fixe, **vérifier activement** que les conditions sont remplies.

### Concept

```yaml
# ❌ AVANT: Délais fixes
delay_s: 5.0      # Espère que 5s suffisent
warmup_s: 10.0    # Espère que 10s suffisent

# ✅ APRÈS: Probes actives
probes:
  ready:
    - type: tf_available
      from_frame: base_footprint
      to_frame: odom
      timeout_s: 60  # Timeout max, mais s'arrête dès que c'est prêt!
    - type: topic_publish
      topic: /global_costmap/costmap
      timeout_s: 60
```

### Avantages

| Aspect | Délais Fixes | Probes Actives |
|--------|--------------|----------------|
| **Déterminisme** | ❌ Non | ✅ Oui |
| **Performance** | ❌ Temps perdu | ✅ Optimal |
| **Robustesse** | ❌ Fragile | ✅ Robuste |
| **Portabilité** | ❌ Machine-dépendant | ✅ Universel |

## 📋 Types de Probes Disponibles

### 1. TF Available
Vérifie qu'une transformation TF est disponible:

```yaml
- type: tf_available
  from_frame: base_footprint
  to_frame: map
  timeout_s: 60
```

**Usage**: S'assurer que le robot est localisé dans la carte.

### 2. Topic Publish
Vérifie qu'un topic publie des messages:

```yaml
- type: topic_publish
  topic: /global_costmap/costmap
  msg_type: nav_msgs/msg/OccupancyGrid
  min_messages: 1
  timeout_s: 60
```

**Usage**: S'assurer que Nav2 publie le costmap.

### 3. Topic Hz
Vérifie qu'un topic publie à une fréquence minimale:

```yaml
- type: topic_hz
  topic: /scan
  msg_type: sensor_msgs/msg/LaserScan
  min_hz: 5.0
  window_s: 5
  timeout_s: 20
```

**Usage**: S'assurer que le lidar fonctionne correctement.

### 4. Service Available
Vérifie qu'un service est disponible:

```yaml
- type: service_available
  service: /navigate_to_pose
  srv_type: nav2_msgs/srv/NavigateToPose
  timeout_s: 60
```

**Usage**: S'assurer que Nav2 est prêt à recevoir des goals.

### 5. Node Present
Vérifie qu'un nœud ROS est actif:

```yaml
- type: node_present
  node: /global_costmap
  timeout_s: 60
```

**Usage**: S'assurer qu'un nœud spécifique est lancé.

## 🔧 Configuration Recommandée

### Pour l'Exploration avec Nav2

```yaml
defaults:
  run:
    warmup_s: 3.0  # Juste pour stabiliser les métriques
    drain_s: 1.0
    timeout_s: 180.0
  
  probes:
    required:
      # 1. Vérifier que le robot reçoit des données lidar
      - type: topic_publish
        topic: /scan
        msg_type: sensor_msgs/msg/LaserScan
        min_messages: 1
        timeout_s: 60
      
      # 2. Vérifier que SLAM publie la carte
      - type: topic_publish
        topic: /map
        msg_type: nav_msgs/msg/OccupancyGrid
        min_messages: 1
        timeout_s: 60
      
      # 3. Vérifier que les TF sont connectées
      - type: tf_available
        from_frame: map
        to_frame: odom
        timeout_s: 60
      
      - type: tf_available
        from_frame: base_footprint
        to_frame: odom
        timeout_s: 60
      
      # 4. Vérifier que Nav2 publie le costmap global
      - type: topic_publish
        topic: /global_costmap/costmap
        msg_type: nav_msgs/msg/OccupancyGrid
        min_messages: 1
        timeout_s: 60

datasets:
  - id: my_dataset
    scenario:
      processes:
        - name: nav2_sim
          # Pas de delay_s, les probes garantissent la synchronisation
          cmd: [...]
        
        - name: explore
          delay_s: 2.0  # Petit délai pour éviter race conditions
          cmd: [...]
```

## 📊 Chronologie avec Probes

```
t=0s    : Démarrage de nav2_sim
t=2s    : Démarrage de l'explorer (delay_s: 2.0)
t=4s    : Tous les processus démarrés (wait 2s orchestrateur)
t=4s    : set_explore(False) - Pause

t=4s    : PROBES START ← Vérifications actives!
  ├─ Attente /scan... ✓ (prêt à t=5s)
  ├─ Attente /map... ✓ (prêt à t=8s)
  ├─ Attente TF map→odom... ✓ (prêt à t=8s)
  ├─ Attente TF base_footprint→odom... ✓ (prêt à t=6s)
  └─ Attente /global_costmap/costmap... ✓ (prêt à t=12s)

t=12s   : TOUTES LES PROBES OK! ← Système vraiment prêt
t=12s   : Warmup 3s (stabilisation métriques)
t=15s   : set_explore(True) - EXPLORATION COMMENCE ✅
```

**Temps total**: 15s (au lieu de 17s avec délais fixes)
**Garantie**: Système vraiment prêt (pas juste "on espère")

## 🎯 Résultat

- ✅ **Déterministe**: Démarre toujours quand c'est vraiment prêt
- ✅ **Rapide**: Pas de temps perdu
- ✅ **Robuste**: Fonctionne sur toutes les machines
- ✅ **Débogable**: Les logs montrent exactement ce qui est attendu

## 💡 Bonnes Pratiques

### 1. Toujours utiliser des probes pour les dépendances critiques

```yaml
# ✅ BON
probes:
  required:
    - type: topic_publish
      topic: /critical_topic
      timeout_s: 60

# ❌ MAUVAIS
delay_s: 10.0  # On espère que ça suffit
```

### 2. Utiliser des timeouts généreux

```yaml
# ✅ BON
timeout_s: 60  # Laisse le temps même sur machine lente

# ❌ MAUVAIS
timeout_s: 5  # Trop court, peut échouer sur machine lente
```

### 3. Combiner probes et petits délais

```yaml
# ✅ BON
probes:
  required: [...]  # Garantit que tout est prêt

processes:
  - name: explore
    delay_s: 2.0  # Petit délai pour éviter race conditions
```

### 4. Utiliser warmup pour stabiliser, pas pour synchroniser

```yaml
# ✅ BON
warmup_s: 3.0  # Stabilise les métriques après que probes confirment que c'est prêt

# ❌ MAUVAIS
warmup_s: 30.0  # Utilisé pour "espérer" que tout est prêt
```

## 🚀 Migration

### Avant
```yaml
defaults:
  run:
    warmup_s: 10.0  # Espère que tout est prêt

datasets:
  - scenario:
      processes:
        - name: nav2
          cmd: [...]
        - name: explore
          delay_s: 15.0  # Espère que Nav2 est prêt
          cmd: [...]
```

### Après
```yaml
defaults:
  run:
    warmup_s: 3.0  # Juste pour stabiliser
  
  probes:
    required:
      - type: topic_publish
        topic: /global_costmap/costmap
        timeout_s: 60

datasets:
  - scenario:
      processes:
        - name: nav2
          cmd: [...]
        - name: explore
          delay_s: 2.0  # Petit délai, probes garantissent Nav2 prêt
          cmd: [...]
```
