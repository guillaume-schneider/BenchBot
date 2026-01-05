"""
Example usage of the centralized logging system.

This file demonstrates all the logging features available in the
SLAM Bench Orchestrator.
"""

from utils.logger import get_logger, LogContext, log_exceptions, generate_crash_report


# ============================================================================
# Example 1: Basic Logging
# ============================================================================

def example_basic_logging():
    """Demonstrate basic logging at different levels."""
    logger = get_logger("example.basic")
    
    logger.debug("This is a debug message (only in file)")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
    logger.critical("This is a critical message")


# ============================================================================
# Example 2: Using LogContext
# ============================================================================

def example_log_context():
    """Demonstrate using LogContext for operation tracking."""
    logger = get_logger("example.context")
    
    with LogContext("Processing Benchmark", logger) as ctx:
        ctx.info("Loading configuration...")
        ctx.info("Starting simulation...")
        ctx.info("Running SLAM algorithm...")
        ctx.info("Collecting metrics...")
        ctx.info("Benchmark completed successfully")


# ============================================================================
# Example 3: Exception Handling with Decorator
# ============================================================================

@log_exceptions()
def example_decorated_function():
    """This function will automatically log any exceptions."""
    logger = get_logger("example.decorated")
    logger.info("Starting risky operation...")
    
    # This will raise an exception and be automatically logged
    result = 10 / 0  # ZeroDivisionError
    
    return result


# ============================================================================
# Example 4: Manual Crash Report
# ============================================================================

def example_manual_crash_report():
    """Demonstrate manual crash report generation."""
    try:
        # Simulate a crash
        raise ValueError("Something went wrong with the configuration!")
    except ValueError as e:
        # Manually generate crash report with context
        context = {
            "config_file": "configs/test.yaml",
            "slam_algorithm": "gmapping",
            "step": "initialization"
        }
        crash_file = generate_crash_report(e, context)
        print(f"Crash report saved to: {crash_file}")


# ============================================================================
# Example 5: Nested LogContext
# ============================================================================

def example_nested_context():
    """Demonstrate nested LogContext for hierarchical operations."""
    logger = get_logger("example.nested")
    
    with LogContext("Full Benchmark Suite", logger) as suite_ctx:
        suite_ctx.info("Starting benchmark suite with 3 runs")
        
        for i in range(3):
            with LogContext(f"Run {i+1}", logger) as run_ctx:
                run_ctx.info("Initializing...")
                run_ctx.info("Executing...")
                run_ctx.info("Completed")
        
        suite_ctx.info("All runs completed")


# ============================================================================
# Example 6: Error Handling in Context
# ============================================================================

def example_context_with_error():
    """Demonstrate automatic error handling in LogContext."""
    logger = get_logger("example.error_context")
    
    try:
        with LogContext("Operation That Fails", logger) as ctx:
            ctx.info("Step 1: Success")
            ctx.info("Step 2: Success")
            # This will trigger automatic crash report
            raise RuntimeError("Step 3 failed!")
    except RuntimeError:
        pass  # Exception was logged and crash report generated


# ============================================================================
# Main Demo
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("SLAM Bench Orchestrator - Logging Examples")
    print("=" * 60)
    print()
    
    print("1. Basic Logging")
    print("-" * 60)
    example_basic_logging()
    print()
    
    print("2. LogContext Usage")
    print("-" * 60)
    example_log_context()
    print()
    
    print("3. Nested LogContext")
    print("-" * 60)
    example_nested_context()
    print()
    
    print("4. Manual Crash Report")
    print("-" * 60)
    example_manual_crash_report()
    print()
    
    print("5. Context with Error (auto crash report)")
    print("-" * 60)
    example_context_with_error()
    print()
    
    print("6. Decorated Function (auto exception logging)")
    print("-" * 60)
    try:
        example_decorated_function()
    except ZeroDivisionError:
        print("Exception was caught and logged!")
    print()
    
    print("=" * 60)
    print("Check logs/orchestrator.log for detailed logs")
    print("Check logs/crashes/ for crash reports")
    print("=" * 60)
