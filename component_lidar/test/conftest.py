"""Tests import this package's source (src/), not the installed copy."""
import os
import sys

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE)
for name in [m for m in sys.modules if m == "src" or m.startswith("src.")]:
    del sys.modules[name]
