
import sys
from pathlib import Path
import re

def main():
    if len(sys.argv) < 3:
        print("Usage: fix_world_file.py <input.sdf> <output.world>")
        sys.exit(1)

    input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])

    with open(input_path, 'r') as f:
        content = f.read()

    # Naive extraction of <model ...> ... </model>
    # We look for the first <model and the last </model>
    # This might be fragile if multiple models, but for this case it's likely single.
    
    start_match = re.search(r'<model\s+name=[\'"]([^\'"]+)[\'"]>', content)
    if not start_match:
        print("[ERROR] No <model> tag found in input file.")
        sys.exit(1)
        
    start_idx = start_match.start()
    end_idx = content.rfind('</model>')
    
    if end_idx == -1:
         print("[ERROR] No closing </model> tag found.")
         sys.exit(1)
         
    model_content = content[start_idx : end_idx + 8] # +8 for </model>

    world_template = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    {model_content}

  </world>
</sdf>
"""

    with open(output_path, 'w') as f:
        f.write(world_template)
    
    print(f"[SUCCESS] Converted {input_path} to valid world file {output_path}")

if __name__ == "__main__":
    main()
