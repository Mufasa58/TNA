import sys
import importlib

MODULE_DIR = "/Users/mmg/dev/tna_playground/"

# 1. Hard-assert the path exists
assert MODULE_DIR and MODULE_DIR.startswith("/"), "Bad module path"
print("Using module dir:", MODULE_DIR)

# 2. Force it to the FRONT of sys.path
if MODULE_DIR in sys.path:
    sys.path.remove(MODULE_DIR)
sys.path.insert(0, MODULE_DIR)

# 3. Import + force reload
import thrust_grid
importlib.reload(thrust_grid)

# 4. Verify exactly which file is used
print("thrust_grid loaded from:", thrust_grid.__file__)

# 5. Run
result = thrust_grid.run()
print("run() returned:", result)
