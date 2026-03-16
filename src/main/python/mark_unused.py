import json
import os
import argparse

# --- PATH CONFIGURATION ---
# Script is in src/main/python/
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_PP_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'deploy', 'pathplanner'))
AUTO_DIR = os.path.join(BASE_PP_DIR, 'autos')
PATH_DIR = os.path.join(BASE_PP_DIR, 'paths')

def find_path_names(command_obj, used_set):
    """Recursively searches the auto JSON for any path references."""
    if not isinstance(command_obj, dict): 
        return
    
    # If this is a path command, save the name
    if command_obj.get("type") == "path" and "data" in command_obj:
        path_name = command_obj["data"].get("pathName")
        if path_name:
            used_set.add(path_name)
    
    # Check for nested commands (Sequential, Parallel, etc.)
    if "data" in command_obj and "commands" in command_obj["data"]:
        for child in command_obj["data"]["commands"]:
            find_path_names(child, used_set)

def run_cleanup(apply_changes):
    if not os.path.exists(AUTO_DIR) or not os.path.exists(PATH_DIR):
        print("❌ Error: Could not find 'autos' or 'paths' folders.")
        print(f"Looked in: {BASE_PP_DIR}")
        return

    print("\n🔍 Scanning Autos for used paths...")
    used_paths = set()
    
    # 1. Collect every path mentioned in an .auto file
    for filename in os.listdir(AUTO_DIR):
        if filename.endswith('.auto'):
            with open(os.path.join(AUTO_DIR, filename), 'r') as f:
                data = json.load(f)
                find_path_names(data.get("command", {}), used_paths)

    print(f"✅ Found {len(used_paths)} paths currently in use.")
    print("-" * 40)

    unused_count = 0
    
    # 2. Check every .path file to see if it's in the 'used' set
    for filename in os.listdir(PATH_DIR):
        if filename.endswith('.path'):
            path_name = os.path.splitext(filename)[0]
            
            if path_name not in used_paths:
                unused_count += 1
                if apply_changes:
                    # Update the file
                    path_file_path = os.path.join(PATH_DIR, filename)
                    with open(path_file_path, 'r+') as f:
                        data = json.load(f)
                        data["folder"] = "unused"
                        f.seek(0)
                        json.dump(data, f, indent=2)
                        f.truncate()
                    print(f"📦 MARKED UNUSED: {filename}")
                else:
                    print(f"💡 WOULD MARK UNUSED: {filename}")

    print("-" * 40)
    if unused_count == 0:
        print("🎉 High five! All your paths are currently being used.")
    elif not apply_changes:
        print(f"Found {unused_count} unused paths. Run with '--do-it' to hide them in PathPlanner.")
    else:
        print(f"Done! {unused_count} paths have been moved to the 'unused' folder in PathPlanner.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="FRC Path Cleanup: Finds paths not used in any Auto and marks them as 'unused'.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    parser.add_argument(
        "--do-it", 
        action="store_true", 
        help="Actually update the files. Without this flag, the script only shows a preview."
    )
    
    args = parser.parse_args()
    run_cleanup(args.do_it)