import os

# Directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

for filename in os.listdir(script_dir):
    if filename.endswith('.sby'):
        file_path = os.path.join(script_dir, filename)
        with open(file_path, 'r') as f:
            lines = f.readlines()

        # Only modify if the file has at least 6 lines
        if len(lines) >= 6:
            # Process line 5 (index 4) and line 6 (index 5)
            try:
                depth_value = int(lines[4].split()[1])
                skip_value = int(lines[5].split()[1])
                lines[4] = f"depth {max(0, depth_value - 1)}\n"
                lines[5] = f"skip {max(0, skip_value - 1)}\n"
            except (IndexError, ValueError):
                print(f"Skipped (format issue): {filename}")
                continue

            with open(file_path, 'w') as f:
                f.writelines(lines)
            print(f"Updated: {filename}")
        else:
            print(f"Skipped (too few lines): {filename}")
