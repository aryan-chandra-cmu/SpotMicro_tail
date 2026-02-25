import trimesh

# adjust these paths
import os
HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "meshes")   # -> spotmicro/assets/meshes
# solid PLA density ~1240 kg/m^3
PLA_SOLID_DENSITY = 1240.0

# choose effective density for 20% infill
# conservative: 0.20 * 1240 = 248
# more realistic with shells: try 450
RHO_EFF = 450.0

mesh_files = [f for f in os.listdir(MESH_DIR) if f.lower().endswith(".stl")]

total_volume_m3 = 0.0
for f in mesh_files:
    path = os.path.join(MESH_DIR, f)
    m = trimesh.load_mesh(path, force='mesh')
    # trimesh volume is in units^3 of the mesh. Your MJCF scales by 0.001,
    # so we must scale the mesh geometry by 0.001 to convert mm->m before volume.
    # If your STL is already in meters, set SCALE = 1.0.
    SCALE = 0.001
    m.apply_scale(SCALE)
    if not m.is_watertight:
        print(f"WARNING: {f} not watertight; volume may be wrong.")
    total_volume_m3 += abs(m.volume)

mass_from_print = RHO_EFF * total_volume_m3
payload_mass = 0.5 + 1.0  # battery+electronics + misc payload
total_mass = mass_from_print + payload_mass

print("Total mesh volume (m^3):", total_volume_m3)
print("Mass from printed parts (kg):", mass_from_print)
print("Total mass incl. payload (kg):", total_mass)
print("Weight (N):", total_mass * 9.81)