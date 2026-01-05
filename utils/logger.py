"""
Centralized logging system for SLAM Bench Orchestrator.

Provides:
- Rotating file logging
- Console logging with colors
- Crash report generation
- Context-aware logging (module, function, line number)
"""

import logging
import logging.handlers
import sys
import traceback
from pathlib import Path
from datetime import datetime
import json


# ============================================================================
# Configuration
# ============================================================================

LOG_DIR = Path(__file__).parent.parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

LOG_FILE = LOG_DIR / "orchestrator.log"
CRASH_REPORT_DIR = LOG_DIR / "crashes"
CRASH_REPORT_DIR.mkdir(exist_ok=True)

# Log levels
DEFAULT_LEVEL = logging.INFO
FILE_LEVEL = logging.DEBUG
CONSOLE_LEVEL = logging.INFO


# ============================================================================
# Color Formatting for Console
# ============================================================================

class ColoredFormatter(logging.Formatter):
    """Custom formatter with color support for console output."""
    
    # ANSI color codes
    COLORS = {
        'DEBUG': '\033[36m',      # Cyan
        'INFO': '\033[32m',       # Green
        'WARNING': '\033[33m',    # Yellow
        'ERROR': '\033[31m',      # Red
        'CRITICAL': '\033[35m',   # Magenta
        'RESET': '\033[0m'        # Reset
    }
    
    def format(self, record):
        # Add color to level name
        levelname = record.levelname
        if levelname in self.COLORS:
            record.levelname = f"{self.COLORS[levelname]}{levelname}{self.COLORS['RESET']}"
        
        return super().format(record)


# ============================================================================
# Logger Setup
# ============================================================================

def setup_logger(name: str = "slam_bench", level: int = DEFAULT_LEVEL) -> logging.Logger:
    """
    Setup a logger with file and console handlers.
    
    Args:
        name: Logger name (usually module name)
        level: Logging level
    
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Avoid duplicate handlers
    if logger.handlers:
        return logger
    
    # File handler with rotation (10MB max, keep 5 backups)
    file_handler = logging.handlers.RotatingFileHandler(
        LOG_FILE,
        maxBytes=10 * 1024 * 1024,  # 10 MB
        backupCount=5,
        encoding='utf-8'
    )
    file_handler.setLevel(FILE_LEVEL)
    file_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(file_formatter)
    
    # Console handler with colors
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(CONSOLE_LEVEL)
    console_formatter = ColoredFormatter(
        '%(levelname)s - %(name)s - %(message)s'
    )
    console_handler.setFormatter(console_formatter)
    
    # Add handlers
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


# ============================================================================
# Crash Report Generation
# ============================================================================

def generate_crash_report(exception: Exception, context: dict = None):
    """
    Generate a detailed crash report and save to file.
    
    Args:
        exception: The exception that caused the crash
        context: Additional context information (config, state, etc.)
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    crash_file = CRASH_REPORT_DIR / f"crash_{timestamp}.json"
    
    # Collect crash information
    crash_data = {
        "timestamp": datetime.now().isoformat(),
        "exception_type": type(exception).__name__,
        "exception_message": str(exception),
        "traceback": traceback.format_exc(),
        "context": context or {},
        "system_info": {
            "python_version": sys.version,
            "platform": sys.platform,
        }
    }
    
    # Save to JSON file
    with open(crash_file, 'w') as f:
        json.dump(crash_data, f, indent=2)
    
    # Also log to main log
    logger = logging.getLogger("slam_bench.crash")
    logger.critical(f"CRASH REPORT GENERATED: {crash_file}")
    logger.critical(f"Exception: {type(exception).__name__}: {exception}")
    logger.critical(f"Traceback:\n{traceback.format_exc()}")
    
    return crash_file


# ============================================================================
# Exception Handler Decorator
# ============================================================================

def log_exceptions(logger: logging.Logger = None, crash_report: bool = True):
    """
    Decorator to automatically log exceptions and generate crash reports.
    
    Args:
        logger: Logger instance to use (creates one if None)
        crash_report: Whether to generate a crash report file
    
    Example:
        @log_exceptions()
        def my_function():
            # This will automatically log any exceptions
            risky_operation()
    """
    if logger is None:
        logger = setup_logger("slam_bench.decorated")
    
    def decorator(func):
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                logger.error(f"Exception in {func.__name__}: {e}", exc_info=True)
                
                if crash_report:
                    context = {
                        "function": func.__name__,
                        "args": str(args)[:200],  # Truncate long args
                        "kwargs": str(kwargs)[:200]
                    }
                    generate_crash_report(e, context)
                
                raise  # Re-raise the exception
        
        return wrapper
    return decorator


# ============================================================================
# Context Manager for Logging
# ============================================================================

class LogContext:
    """
    Context manager for logging operations with automatic error handling.
    
    Example:
        with LogContext("Processing benchmark", logger) as ctx:
            ctx.info("Starting processing...")
            risky_operation()
            ctx.info("Completed successfully")
    """
    
    def __init__(self, operation_name: str, logger: logging.Logger = None):
        self.operation_name = operation_name
        self.logger = logger or setup_logger("slam_bench.context")
        self.start_time = None
    
    def __enter__(self):
        self.start_time = datetime.now()
        self.logger.info(f"[START] {self.operation_name}")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        duration = (datetime.now() - self.start_time).total_seconds()
        
        if exc_type is None:
            self.logger.info(f"[SUCCESS] {self.operation_name} (took {duration:.2f}s)")
        else:
            self.logger.error(
                f"[FAILED] {self.operation_name} (took {duration:.2f}s): {exc_val}",
                exc_info=True
            )
            
            # Generate crash report
            context = {
                "operation": self.operation_name,
                "duration_seconds": duration
            }
            generate_crash_report(exc_val, context)
        
        # Don't suppress the exception
        return False
    
    def info(self, message: str):
        """Log info message."""
        self.logger.info(f"[{self.operation_name}] {message}")
    
    def debug(self, message: str):
        """Log debug message."""
        self.logger.debug(f"[{self.operation_name}] {message}")
    
    def warning(self, message: str):
        """Log warning message."""
        self.logger.warning(f"[{self.operation_name}] {message}")
    
    def error(self, message: str):
        """Log error message."""
        self.logger.error(f"[{self.operation_name}] {message}")


# ============================================================================
# Global Logger Instance
# ============================================================================

# Create default logger
default_logger = setup_logger("slam_bench")


# ============================================================================
# Convenience Functions
# ============================================================================

def get_logger(name: str = None) -> logging.Logger:
    """
    Get a logger instance.
    
    Args:
        name: Logger name (uses 'slam_bench' if None)
    
    Returns:
        Logger instance
    """
    if name is None:
        return default_logger
    return setup_logger(f"slam_bench.{name}")


def log_system_info():
    """Log system information at startup."""
    logger = get_logger("system")
    logger.info("=" * 60)
    logger.info("SLAM Bench Orchestrator - System Information")
    logger.info("=" * 60)
    logger.info(f"Python Version: {sys.version}")
    logger.info(f"Platform: {sys.platform}")
    logger.info(f"Log File: {LOG_FILE}")
    logger.info(f"Crash Reports: {CRASH_REPORT_DIR}")
    logger.info("=" * 60)


# ============================================================================
# Example Usage
# ============================================================================

if __name__ == "__main__":
    # Example 1: Basic logging
    logger = get_logger("example")
    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
    
    # Example 2: Using decorator
    @log_exceptions()
    def risky_function():
        raise ValueError("Something went wrong!")
    
    try:
        risky_function()
    except ValueError:
        pass  # Exception was logged
    
    # Example 3: Using context manager
    with LogContext("Example Operation") as ctx:
        ctx.info("Step 1: Loading data")
        ctx.info("Step 2: Processing")
        ctx.info("Step 3: Saving results")
    
    print(f"\nLog file created at: {LOG_FILE}")
    print(f"Check crash reports in: {CRASH_REPORT_DIR}")
