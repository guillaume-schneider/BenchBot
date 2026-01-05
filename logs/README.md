# Logs Directory

This directory contains all log files and crash reports for the SLAM Bench Orchestrator.

## 📁 Structure

```
logs/
├── orchestrator.log         # Main rotating log file
├── orchestrator.log.1       # Backup log (rotated)
├── orchestrator.log.2       # Older backup
├── ...
└── crashes/                 # Crash reports directory
    ├── crash_20260105_061500.json
    └── crash_20260105_062300.json
```

## 📝 Log Files

### Main Log (`orchestrator.log`)
- **Max Size:** 10 MB
- **Rotation:** Automatic (keeps 5 backups)
- **Format:** `YYYY-MM-DD HH:MM:SS - module - LEVEL - file:line - message`
- **Levels:** DEBUG, INFO, WARNING, ERROR, CRITICAL

### Example Log Entry
```
2026-01-05 06:15:30 - slam_bench.orchestrator - INFO - orchestrator.py:95 - O3DE found at: /opt/o3de
2026-01-05 06:15:31 - slam_bench.metrics - DEBUG - metrics.py:342 - Computing SSIM for maps
2026-01-05 06:15:32 - slam_bench.orchestrator - ERROR - orchestrator.py:120 - Failed to convert SDF: FileNotFoundError
```

## 🐛 Crash Reports

Crash reports are automatically generated when unhandled exceptions occur.

### Crash Report Format (JSON)
```json
{
  "timestamp": "2026-01-05T06:15:00",
  "exception_type": "ValueError",
  "exception_message": "Invalid configuration",
  "traceback": "Traceback (most recent call last):\n  File ...",
  "context": {
    "operation": "Loading configuration",
    "duration_seconds": 0.5
  },
  "system_info": {
    "python_version": "3.10.12",
    "platform": "linux"
  }
}
```

## 🔍 Viewing Logs

### Tail Live Logs
```bash
tail -f logs/orchestrator.log
```

### Search for Errors
```bash
grep "ERROR" logs/orchestrator.log
```

### View Recent Crashes
```bash
ls -lt logs/crashes/ | head
```

### Analyze Crash Report
```bash
cat logs/crashes/crash_20260105_061500.json | jq .
```

## 🧹 Cleanup

Logs are automatically rotated, but you can manually clean old files:

```bash
# Remove old backups (keep only .log and .log.1)
rm logs/orchestrator.log.[2-9]

# Remove crash reports older than 30 days
find logs/crashes/ -name "*.json" -mtime +30 -delete
```

## ⚙️ Configuration

Log settings can be modified in `utils/logger.py`:

```python
# Log levels
DEFAULT_LEVEL = logging.INFO
FILE_LEVEL = logging.DEBUG
CONSOLE_LEVEL = logging.INFO

# Rotation settings
maxBytes=10 * 1024 * 1024  # 10 MB
backupCount=5               # Keep 5 backups
```

## 🚨 Troubleshooting

### No Logs Generated
- Check file permissions: `ls -la logs/`
- Verify logger is imported: `from utils.logger import get_logger`

### Logs Too Verbose
- Increase console level to WARNING:
  ```python
  CONSOLE_LEVEL = logging.WARNING
  ```

### Disk Space Issues
- Reduce `maxBytes` or `backupCount` in `utils/logger.py`
- Set up automatic cleanup cron job

---

**Note:** This directory is automatically created and managed by the logging system. Do not manually edit log files while the application is running.
