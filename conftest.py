import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src"))  # run tests without `pip install -e .`
os.environ["PILLBOT_NO_LOCAL"] = "1"                    # tests use the repo configs, not your local overrides
