"""Project-local Python startup hooks.

This repository is used in an environment that preloads ROS launch testing
plugins. Those plugins are not always installed with their optional runtime
requirements, so ``pytest`` may crash before collection while scanning the
installed ``pytest11`` entry points.

When working from this project tree, disable third-party pytest plugin
auto-loading so local validation and hidden tests run against the repository's
own Python code, not the ROS environment.
"""

from __future__ import annotations

import os
from pathlib import Path


def _is_project_root(start: str) -> bool:
    current = Path(start).resolve()
    for candidate in (current, *current.parents):
        root = candidate
        if (root / "README.md").is_file() and (root / "scripts").is_dir():
            return True
    return False


if _is_project_root(os.getcwd()):
    os.environ.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
