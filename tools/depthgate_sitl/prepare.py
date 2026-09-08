"""Patch only an isolated CrazySim snapshot to compile the active controller."""
from pathlib import Path
import sys
p=Path(sys.argv[1])/'sitl_make/CMakeLists.txt'
s=p.read_text()
s=s.replace('sequential_obstacle_link_sitl.c','espnet_collision_link_sitl.c')
s=s.replace('    ${TINYMPC_APP_DIR}/src/tinyracer_racing.c\n','')
s=s.replace('    ${TINYMPC_APP_DIR}/src/tinyracer_debug.c\n','')
p.write_text(s)
