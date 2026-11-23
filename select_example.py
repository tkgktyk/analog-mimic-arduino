Import("env")
import os

example = os.environ.get("MIMIC_EXAMPLE") or env.GetProjectOption("custom_example", "")
if not example:
    raise SystemExit("No example selected. Set custom_example or MIMIC_EXAMPLE.")
root = os.path.join(env.subst("$PROJECT_DIR"), "examples")
src = os.path.join(root, example)
if not os.path.isdir(src):
    # The examples are grouped in folders (01.Basics/BasicRead): find the sketch by its name
    found = [os.path.join(d, example) for d, dirs, _ in os.walk(root) if example in dirs]
    if len(found) != 1:
        raise SystemExit(f"Example not found (or not unique): {example}")
    src = found[0]
env.Replace(PROJECT_SRC_DIR=src)
print(f"*** Building example: {example}")
