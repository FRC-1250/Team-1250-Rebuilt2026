import json
import os
import argparse

# --- PATH CONFIGURATION ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_PP_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'deploy', 'pathplanner'))
AUTO_DIR = os.path.join(BASE_PP_DIR, 'autos')
PATH_DIR = os.path.join(BASE_PP_DIR, 'paths')
FIELD_WIDTH = 8.21

def swap_side_string(text, direction):
    """
    Swaps 'Left' for 'Right' (or vice versa) while keeping the same 
    Capitalization (e.g., Left -> Right, left -> right).
    """
    if not text:
        return text
    
    src, dest = ("Left", "Right") if direction == "LtoR" else ("Right", "Left")
    
    # Handle Title Case (Left -> Right)
    text = text.replace(src, dest)
    # Handle lowercase (left -> right)
    text = text.replace(src.lower(), dest.lower())
    
    return text

def mirror_y_coords(obj):
    if isinstance(obj, dict):
        if "y" in obj and isinstance(obj["y"], (int, float)):
            obj["y"] = FIELD_WIDTH - obj["y"]
        for key in obj:
            mirror_y_coords(obj[key])
    elif isinstance(obj, list):
        for item in obj:
            mirror_y_coords(item)

def mirror_path_file(path_name, direction):
    old_filename = f"{path_name}.path"
    old_path = os.path.join(PATH_DIR, old_filename)
    
    if not os.path.exists(old_path):
        print(f"  ⚠️  Skipping: {old_filename} (File not found)")
        return None

    with open(old_path, 'r') as f:
        data = json.load(f)

    # 1. Flip all Y coordinates
    mirror_y_coords(data)

    # 2. Flip all rotations
    for rt in data.get("rotationTargets", []):
        rt["rotationDegrees"] *= -1
    for key in ["goalEndState", "idealStartingState"]:
        if key in data:
            data[key]["rotation"] *= -1

    # 3. FIX: Update linked waypoint names (e.g., 'LeftStation' -> 'RightStation')
    for wp in data.get("waypoints", []):
        if wp.get("linkedName"):
            original_link = wp["linkedName"]
            wp["linkedName"] = swap_side_string(original_link, direction)
            if original_link != wp["linkedName"]:
                print(f"    🔗 Updated link: {original_link} -> {wp['linkedName']}")

    # 4. Determine the new filename
    new_path_name = swap_side_string(path_name, direction)
    if new_path_name == path_name:
        new_path_name = f"{path_name}_mirrored"

    with open(os.path.join(PATH_DIR, f"{new_path_name}.path"), 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"  ✅ Path Mirrored: {path_name} -> {new_path_name}")
    return new_path_name

def process_auto(auto_filename, direction):
    if not auto_filename.endswith('.auto'):
        auto_filename += '.auto'

    auto_path = os.path.join(AUTO_DIR, auto_filename)
    if not os.path.exists(auto_path):
        print(f"❌ Error: Could not find '{auto_filename}'")
        return

    print(f"\n🚀 Mirroring Auto: {auto_filename} ({direction})")
    print("-" * 40)

    with open(auto_path, 'r') as f:
        auto_data = json.load(f)

    def update_paths(command_obj):
        if not isinstance(command_obj, dict): return
        if command_obj.get("type") == "path":
            old_name = command_obj["data"].get("pathName")
            if old_name:
                new_name = mirror_path_file(old_name, direction)
                if new_name:
                    command_obj["data"]["pathName"] = new_name
        
        if "data" in command_obj and "commands" in command_obj["data"]:
            for child in command_obj["data"]["commands"]:
                update_paths(child)

    update_paths(auto_data.get("command", {}))

    # Rename the Auto file itself
    new_auto_name = swap_side_string(auto_filename, direction)
    if new_auto_name == auto_filename:
        new_auto_name = f"mirrored_{auto_filename}"

    with open(os.path.join(AUTO_DIR, new_auto_name), 'w') as f:
        json.dump(auto_data, f, indent=2)
    
    print("-" * 40)
    print(f"🎉 SUCCESS! New Auto created: {new_auto_name}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="FRC Path Mirror Tool: Flips an auto and updates all linked waypoints.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("filename", help="The name of your .auto file")
    parser.add_argument("direction", choices=["LtoR", "RtoL"], help="LtoR: Left to Right\nRtoL: Flip from Right to Left")
    
    args = parser.parse_args()
    process_auto(args.filename, args.direction)