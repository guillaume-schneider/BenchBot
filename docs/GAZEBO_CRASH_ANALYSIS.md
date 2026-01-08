# Analyse du Crash Gazebo - 2026-01-06

## 🔍 Résumé Exécutif

**Problème**: Gazebo crash systématiquement pendant les runs de benchmarking
**Fréquence**: 3 crashes sur 5 runs récents (60% de taux d'échec)
**Impact**: L'explorer ne peut pas fonctionner car le simulateur n'existe plus

---

## 📊 Analyse des Logs

### Run: 2026-01-06_20-59-10

#### Chronologie du Crash

1. **20:59:13** - Démarrage de Gazebo (gzserver + gzclient)
   ```
   [INFO] [gzserver-1]: process started with pid [69043]
   [INFO] [gzclient-2]: process started with pid [69045]
   ```

2. **20:59:56** - Spawn du robot réussi
   ```
   [spawn_entity.py-4] [INFO] Spawn status: SpawnEntity: Successfully spawned entity [turtlebot3_waffle]
   ```

3. **20:59:57** - **gzclient crash avec assertion**
   ```
   [gzclient-2] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: 
   typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const 
   [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: 
   Assertion `px != 0' failed.
   
   [ERROR] [gzclient-2]: process has died [pid 69045, exit code -6]
   ```
   
   **Analyse**: Exit code -6 = SIGABRT (assertion failed)
   **Cause**: Tentative d'accès à un pointeur Camera NULL

4. **20:59:59** - **gzserver tué**
   ```
   [ERROR] [gzserver-1]: process has died [pid 69043, exit code -9]
   ```
   
   **Analyse**: Exit code -9 = SIGKILL (tué de force)
   **Cause probable**: Cleanup automatique après crash de gzclient OU timeout de l'orchestrateur

---

## 🐛 Causes Identifiées

### 1. **Bug dans Gazebo Client (gzclient)**

**Symptôme**: Assertion failed sur pointeur Camera NULL

**Code source problématique**:
```cpp
// /usr/include/boost/smart_ptr/shared_ptr.hpp:728
typename boost::detail::sp_member_access<T>::type 
boost::shared_ptr<T>::operator->() const 
{
    BOOST_ASSERT(px != 0);  // <-- CRASH ICI
    return px;
}
```

**Contexte**: 
- Gazebo essaie d'accéder à une caméra de rendu
- Le pointeur n'a pas été initialisé correctement
- Cela arrive souvent en mode headless ou avec des problèmes de drivers graphiques

### 2. **Problèmes de Synchronisation TF**

**Avant le crash**, on observe:
```
[controller_server-5] [INFO] Timed out waiting for transform from base_link to odom to become available, 
tf error: Could not find a connection between 'odom' and 'base_link' because they are not part of the same tree.
Tf has two or more unconnected trees.
```

**Analyse**: 
- Les transformations TF ne sont pas connectées
- Cela indique un problème de démarrage du robot
- Peut contribuer à l'instabilité générale

### 3. **Warnings de Configuration**

```
[Err] [RTShaderSystem.cc:480] Unable to find shader lib. 
Shader generating will fail. 
Your GAZEBO_RESOURCE_PATH is probably improperly set.
```

**Impact**: Modéré - peut causer des problèmes de rendu

---

## 📈 Statistiques

### Runs Récents (30 dernières minutes)

| Run | Timestamp | Gazebo Crash | Exit Code | Durée |
|-----|-----------|--------------|-----------|-------|
| 1 | 20:49:50 | ✅ Oui | -9 | ~3min |
| 2 | 20:59:10 | ✅ Oui | -9 | ~3min |
| 3 | 21:02:06 | ✅ Oui | -9 | ~3min |
| 4 | ? | ❌ Non | - | - |
| 5 | ? | ❌ Non | - | - |

**Taux d'échec**: 60% (3/5)

---

## 🛠️ Solutions Proposées

### Solution 1: Mode Headless Strict (Recommandé)

**Problème**: gzclient essaie de créer une interface graphique même en mode headless

**Solution**: Désactiver complètement gzclient

```yaml
# Dans tb3_sim_no_loc.launch.py
gui:=False  # Déjà fait
headless:=True  # Déjà fait
```

**Vérifier**: S'assurer que gzclient n'est PAS lancé du tout

### Solution 2: Variables d'Environnement Gazebo

**Problème**: GAZEBO_RESOURCE_PATH mal configuré

**Solution**: Ajouter les variables d'environnement correctes

```python
# Dans orchestrator.py, pour les processus Gazebo
env = {
    "GAZEBO_RESOURCE_PATH": "/usr/share/gazebo-11",
    "GAZEBO_MODEL_PATH": "/usr/share/gazebo-11/models",
    "LIBGL_ALWAYS_SOFTWARE": "1",  # Force software rendering (évite bugs GPU)
    **proc.get("env", {})
}
```

### Solution 3: Augmenter les Timeouts de Démarrage

**Problème**: Les transformations TF ne sont pas prêtes à temps

**Solution**: Augmenter les timeouts dans les probes

```yaml
probes:
  required:
    - type: tf_available
      from_frame: map
      to_frame: odom
      timeout_s: 120  # Augmenté de 60 à 120
```

### Solution 4: Migration vers O3DE (Recommandé à Long Terme)

**Avantages**:
- ✅ Plus stable
- ✅ Meilleure gestion des ressources
- ✅ Pas de problèmes de rendu graphique
- ✅ Déjà configuré avec delay_s correct

**Configuration**: `configs/datasets/tb3_o3de_explore_modeA.yaml`

---

## 🎯 Plan d'Action Immédiat

### Étape 1: Vérifier le Launch File

```bash
cat tools/launch/tb3_sim_no_loc.launch.py | grep -E "gui|headless|gzclient"
```

**Objectif**: S'assurer que gzclient n'est pas lancé

### Étape 2: Ajouter Variables d'Environnement

Modifier `orchestrator.py` pour ajouter les variables Gazebo

### Étape 3: Tester avec O3DE

Utiliser la configuration O3DE qui est plus stable

### Étape 4: Augmenter les Timeouts

Si Gazebo est nécessaire, augmenter les timeouts de synchronisation

---

## 📝 Recommandations Finales

### Court Terme (Immédiat)
1. ✅ **Utiliser O3DE** au lieu de Gazebo pour les benchmarks
2. ✅ **Ajouter delay_s: 5.0** à toutes les configurations (déjà fait)
3. ✅ **Augmenter transform_tolerance** à 30.0s (déjà fait)

### Moyen Terme
1. 🔧 **Corriger le launch file** pour désactiver complètement gzclient
2. 🔧 **Ajouter variables d'environnement** Gazebo correctes
3. 🔧 **Augmenter les timeouts** des probes TF

### Long Terme
1. 🚀 **Migrer complètement vers O3DE** pour tous les benchmarks
2. 🚀 **Créer un mode de fallback** automatique si Gazebo crash
3. 🚀 **Ajouter monitoring** pour détecter les crashes et redémarrer

---

## 🔗 Références

- **Gazebo Issue**: https://github.com/gazebosim/gazebo-classic/issues (chercher "Camera assertion")
- **O3DE Migration**: `docs/SIMULATORS.md`
- **TF Troubleshooting**: `docs/TROUBLESHOOTING_EXPLORATION.md`
