# Contributing to SLAM Bench Orchestrator

Thank you for your interest in contributing to the SLAM Bench Orchestrator! This document provides guidelines and best practices for contributing to the project.

## 🚀 Getting Started

### Prerequisites
- **OS:** Ubuntu 22.04 LTS (recommended)
- **ROS 2:** Humble Hawksbill
- **Python:** 3.10+
- **Git:** For version control

### Setup Development Environment

1. **Clone the repository:**
   ```bash
   git clone https://github.com/YOUR_USERNAME/benchbot.git
   cd benchbot
   ```

2. **Install dependencies:**
   ```bash
   # System dependencies
   ./install.sh
   
   # Python dependencies
   pip3 install -r requirements.txt
   
   # Development tools (optional)
   pip3 install pytest black flake8
   ```

3. **Verify installation:**
   ```bash
   python3 gui/main.py
   ```

---

## 📝 Code Style Guidelines

### Python Code Style

We follow **PEP 8** with some project-specific conventions:

#### Formatting
- **Line Length:** Max 100 characters (not the default 79)
- **Indentation:** 4 spaces (no tabs)
- **Quotes:** Double quotes for strings, single quotes for dict keys

#### Naming Conventions
```python
# Classes: PascalCase
class MetricsCalculator:
    pass

# Functions/Methods: snake_case
def compute_ate_rmse(trajectory):
    pass

# Constants: UPPER_SNAKE_CASE
MAX_RETRY_COUNT = 3

# Private methods: _leading_underscore
def _internal_helper():
    pass
```

#### Docstrings
Use Google-style docstrings:
```python
def align_trajectories(gt_poses, est_poses, method='first_point'):
    """
    Align estimated trajectory to ground truth.
    
    Args:
        gt_poses: List of (x, y) ground truth positions
        est_poses: List of (x, y) estimated positions
        method: Alignment strategy ('first_point', 'umeyama')
    
    Returns:
        Tuple of (aligned_poses, transformation_matrix)
    
    Raises:
        ValueError: If trajectories have different lengths
    """
    pass
```

#### Type Hints
Use type hints for function signatures:
```python
from typing import List, Tuple, Optional

def compute_coverage(gt_map: np.ndarray, est_map: np.ndarray) -> float:
    """Calculate map coverage percentage."""
    pass
```

---

## 🧪 Testing

### Running Tests
```bash
# Run all tests
pytest tests/

# Run specific test file
pytest tests/test_metrics.py

# Run with coverage report
pytest --cov=evaluation --cov=runner tests/
```

### Writing Tests
Place tests in the `tests/` directory with the naming convention `test_*.py`:

```python
# tests/test_metrics.py
import pytest
import numpy as np
from evaluation.metrics import compute_ssim

def test_ssim_identical_maps():
    """SSIM should be 1.0 for identical maps."""
    map_a = np.random.randint(0, 100, (100, 100), dtype=np.int8)
    map_b = map_a.copy()
    
    ssim_score = compute_ssim(map_a, map_b)
    assert ssim_score > 0.99  # Allow for floating point errors

def test_ssim_completely_different():
    """SSIM should be low for completely different maps."""
    map_a = np.zeros((100, 100), dtype=np.int8)
    map_b = np.full((100, 100), 100, dtype=np.int8)
    
    ssim_score = compute_ssim(map_a, map_b)
    assert ssim_score < 0.5
```

---

## 🔀 Git Workflow

### Branch Naming
- **Features:** `feature/auto-tuner-grid-search`
- **Bug Fixes:** `fix/docker-volume-mount`
- **Documentation:** `docs/update-metrics-guide`
- **Refactoring:** `refactor/orchestrator-cleanup`

### Commit Messages
Follow the [Conventional Commits](https://www.conventionalcommits.org/) specification:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, no logic change)
- `refactor`: Code refactoring
- `test`: Adding or updating tests
- `chore`: Maintenance tasks

**Examples:**
```
feat(auto-tuner): implement Bayesian optimization

Add support for Gaussian Process-based parameter optimization
using scikit-optimize. Includes real-time convergence plotting
in the GUI.

Closes #42
```

```
fix(docker): resolve NumPy compatibility issue

Add compatibility patch for tf_transformations with NumPy 2.x
by restoring deprecated np.float and np.int aliases.

Fixes #58
```

---

## 🔧 Pull Request Process

### Before Submitting

1. **Update documentation** if you changed APIs or added features
2. **Add tests** for new functionality
3. **Run the test suite** and ensure all tests pass
4. **Format your code:**
   ```bash
   black gui/ runner/ evaluation/ tools/
   flake8 gui/ runner/ evaluation/ tools/
   ```
5. **Update CHANGELOG.md** with your changes

### PR Template

When creating a PR, use this template:

```markdown
## Description
Brief description of what this PR does.

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update

## Testing
Describe the tests you ran to verify your changes:
- [ ] Unit tests pass (`pytest tests/`)
- [ ] Manual testing in GUI
- [ ] Headless mode tested
- [ ] Docker build successful

## Checklist
- [ ] My code follows the style guidelines
- [ ] I have performed a self-review of my code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally with my changes

## Screenshots (if applicable)
Add screenshots to help explain your changes.

## Related Issues
Closes #(issue number)
```

### Review Process

1. **Automated Checks:** CI/CD will run tests and linting
2. **Code Review:** At least one maintainer will review your code
3. **Feedback:** Address any requested changes
4. **Merge:** Once approved, a maintainer will merge your PR

---

## 🐛 Reporting Bugs

### Before Reporting
- Check if the bug has already been reported in [Issues](https://github.com/YOUR_USERNAME/benchbot/issues)
- Try to reproduce the bug with the latest version

### Bug Report Template
```markdown
**Describe the bug**
A clear and concise description of what the bug is.

**To Reproduce**
Steps to reproduce the behavior:
1. Go to '...'
2. Click on '....'
3. See error

**Expected behavior**
What you expected to happen.

**Screenshots**
If applicable, add screenshots.

**Environment:**
 - OS: [e.g., Ubuntu 22.04]
 - ROS 2 Version: [e.g., Humble]
 - Python Version: [e.g., 3.10.12]
 - Docker: [Yes/No]

**Logs**
Attach relevant log files from `logs/` or terminal output.

**Additional context**
Any other context about the problem.
```

---

## 💡 Feature Requests

We welcome feature suggestions! Please use the [Feature Request template](https://github.com/YOUR_USERNAME/benchbot/issues/new?template=feature_request.md) and include:

- **Use Case:** Why is this feature needed?
- **Proposed Solution:** How should it work?
- **Alternatives:** Other approaches you've considered
- **Priority:** How important is this to you?

---

## 📚 Documentation

### Adding Documentation

- **Code Documentation:** Use docstrings for all public functions/classes
- **User Guides:** Add markdown files to `docs/` for new features
- **Update README:** If your change affects the Quick Start or main features

### Documentation Style
- Use clear, concise language
- Include code examples where applicable
- Add screenshots for GUI features
- Link to related documentation

---

## 🏗️ Project Structure

Understanding the codebase:

```
benchbot/
├── gui/                    # PyQt5 GUI application
│   ├── main.py            # Entry point
│   ├── pages/             # GUI pages (Dashboard, Comparison, etc.)
│   └── worker.py          # Background task runner
├── runner/                # Core orchestration logic
│   ├── orchestrator.py    # Main benchmark runner
│   ├── run_one.py         # Single run executor
│   └── run_matrix.py      # Batch runner
├── evaluation/            # Metrics calculation
│   └── metrics.py         # ATE, SSIM, Coverage, etc.
├── tools/                 # Utility scripts
│   ├── benchmark.py       # ATE calculation
│   └── report_generator.py # PDF reports
├── configs/               # YAML configurations
│   ├── matrices/          # Benchmark suites
│   ├── slams/             # SLAM profiles
│   └── datasets/          # Simulation scenarios
├── results/               # Output directory
└── docs/                  # Documentation
```

---

## 🤝 Community

- **Discussions:** Use [GitHub Discussions](https://github.com/YOUR_USERNAME/benchbot/discussions) for questions
- **Discord:** Join our [Discord server](#) for real-time chat
- **Email:** For private inquiries: maintainer@example.com

---

## 📜 License

By contributing, you agree that your contributions will be licensed under the same license as the project (MIT License).

---

## 🙏 Acknowledgments

Thank you for contributing to making SLAM benchmarking more accessible and reproducible!

**Happy Coding!** 🚀
