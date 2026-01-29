import sys, os, importlib, traceback

MODULE_DIR  = "/Users/mmg/dev/tna_playground"
MODULE_NAME = "thrust_1_bundled"   # <-- CHANGE ONLY HERE (without .py)

assert os.path.isdir(MODULE_DIR), f"MODULE_DIR not found: {MODULE_DIR}"
print("Using module dir:", MODULE_DIR)

if MODULE_DIR in sys.path:
    sys.path.remove(MODULE_DIR)
sys.path.insert(0, MODULE_DIR)

# hard cache-bust
if MODULE_NAME in sys.modules:
    del sys.modules[MODULE_NAME]

try:
    mod = importlib.import_module(MODULE_NAME)
    importlib.reload(mod)

    print(f"{MODULE_NAME} loaded from:", getattr(mod, "__file__", "<no __file__>"))
    print("Has run():", hasattr(mod, "run"))

    # Run with traceback
    result = mod.run()
    print("run() returned:", result)

except Exception:
    print("ERROR while running module:")
    print(traceback.format_exc())
