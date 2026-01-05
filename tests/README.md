# SLAM Bench Orchestrator - Test Suite

This directory contains the unit and integration tests for the SLAM Bench Orchestrator.

## 📁 Test Structure

```
tests/
├── conftest.py              # Pytest configuration and fixtures
├── test_metrics.py          # Metrics calculation tests
├── test_orchestrator.py     # Orchestrator lifecycle tests
├── test_docker.py           # Docker containerization tests
├── test_gui_components.py   # GUI widget tests
└── data/                    # Test data (auto-created)
```

## 🚀 Running Tests

### Run All Tests
```bash
pytest tests/
```

### Run Specific Test File
```bash
pytest tests/test_metrics.py
```

### Run with Verbose Output
```bash
pytest tests/ -v
```

### Run with Coverage Report
```bash
pytest tests/ --cov=evaluation --cov=runner --cov=gui --cov-report=html
```

### Skip Slow Tests
```bash
pytest tests/ -m "not slow"
```

### Run Only Integration Tests
```bash
pytest tests/ -m integration
```

## 🏷️ Test Markers

Tests are marked with the following categories:

- **`@pytest.mark.slow`**: Tests that take >5 seconds (e.g., Docker builds)
- **`@pytest.mark.integration`**: Integration tests requiring external services
- **`@pytest.mark.gui`**: Tests requiring a display/X11 server

## 📊 Test Coverage

### Current Coverage (Target: >80%)

| Module         | Coverage | Status |
|----------------|----------|--------|
| `evaluation/`  | ~85%     | ✅     |
| `runner/`      | ~70%     | 🟡     |
| `gui/`         | ~60%     | 🟡     |
| `tools/`       | ~75%     | ✅     |

## 🧪 Test Categories

### 1. **Metrics Tests** (`test_metrics.py`)
Tests for evaluation metrics:
- ✅ Coverage calculation
- ✅ IoU (Intersection over Union)
- ✅ SSIM (Structural Similarity)
- ✅ Wall thickness analysis
- ✅ Anomaly detection
- ✅ Path length calculation

**Example:**
```bash
pytest tests/test_metrics.py::TestMapQualityMetrics::test_coverage_perfect_match -v
```

### 2. **Orchestrator Tests** (`test_orchestrator.py`)
Tests for run lifecycle:
- ✅ Configuration loading
- ✅ Run ID generation
- ✅ Metrics collection
- ✅ Error handling
- ✅ Path resolution

**Example:**
```bash
pytest tests/test_orchestrator.py::TestConfigurationLoading -v
```

### 3. **Docker Tests** (`test_docker.py`)
Tests for containerization:
- ✅ Dockerfile syntax validation
- ✅ docker-compose.yml validation
- ✅ Volume mounting configuration
- ✅ Environment variables
- 🐌 Docker build (slow test)

**Example:**
```bash
# Skip slow Docker build test
pytest tests/test_docker.py -m "not slow"

# Run full Docker integration tests
pytest tests/test_docker.py -m integration
```

### 4. **GUI Tests** (`test_gui_components.py`)
Tests for PyQt5 widgets:
- ✅ Window initialization
- ✅ Widget creation
- ✅ Signal/slot connections
- ✅ Data validation
- ✅ UI state management

**Example:**
```bash
# Requires DISPLAY environment variable
DISPLAY=:0 pytest tests/test_gui_components.py -v
```

## 🛠️ Writing New Tests

### Test Template
```python
"""
Brief description of what this test file covers.
"""

import pytest
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from your_module import your_function


class TestYourFeature:
    """Test suite for your feature."""
    
    def test_basic_functionality(self):
        """Should do X when Y happens."""
        result = your_function(input_data)
        assert result == expected_output
    
    def test_edge_case(self):
        """Should handle edge case Z."""
        with pytest.raises(ValueError):
            your_function(invalid_input)
```

### Using Fixtures
```python
def test_with_sample_config(sample_config):
    """Use the sample_config fixture from conftest.py."""
    assert sample_config['slam'] == 'gmapping'
```

### Marking Tests
```python
@pytest.mark.slow
def test_expensive_operation():
    """This test takes a long time."""
    pass

@pytest.mark.integration
def test_with_external_service():
    """This test requires Docker/ROS."""
    pass
```

## 🐛 Debugging Failed Tests

### Run Single Test with Full Output
```bash
pytest tests/test_metrics.py::TestMapQualityMetrics::test_coverage_partial -vv -s
```

### Drop into Debugger on Failure
```bash
pytest tests/ --pdb
```

### Show Local Variables on Failure
```bash
pytest tests/ -l
```

## 📈 Continuous Integration

Tests are automatically run on:
- Every commit (via pre-commit hook)
- Every pull request (via GitHub Actions)
- Nightly builds (full test suite including slow tests)

### CI Configuration
See `.github/workflows/ci.yml` for the full CI pipeline.

## 🎯 Test Quality Guidelines

1. **Isolation**: Each test should be independent
2. **Clarity**: Test names should describe what they test
3. **Speed**: Keep tests fast (< 1s) unless marked `@pytest.mark.slow`
4. **Coverage**: Aim for >80% code coverage
5. **Assertions**: Use specific assertions (not just `assert True`)

## 📚 Additional Resources

- [pytest Documentation](https://docs.pytest.org/)
- [pytest-cov Plugin](https://pytest-cov.readthedocs.io/)
- [PyQt5 Testing Guide](https://doc.qt.io/qt-5/qtest-overview.html)

---

**Happy Testing!** 🧪✅
