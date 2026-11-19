# Lets `python3 -m pytest karr_control/test` run from the repo root without building the package.
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
