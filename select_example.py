Import("env")
import os

example = os.environ.get("MIMIC_EXAMPLE") or env.GetProjectOption("custom_example", "")
if not example:
    raise SystemExit("No example selected. Set custom_example or MIMIC_EXAMPLE.")
src = os.path.join(env.subst("$PROJECT_DIR"), "examples", example)
if not os.path.isdir(src):
    raise SystemExit(f"Example not found: {src}")
env.Replace(PROJECT_SRC_DIR=src)
print(f"*** Building example: {example}")
