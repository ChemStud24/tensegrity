import numpy as np
from PIL import Image
import os

# =============================================================================
# CONFIGURATION - Change these values as needed
# =============================================================================
d = 10.0          # Side length of each grid square (meters)
angle_degs = 10.0 # Ramp angle in degrees
W, H = 201, 201   # Heightfield resolution (pixels)

# Heightfield base elevation (meters below ground, for collision thickness)
hfield_z_base = 0.5

# =============================================================================
# PATHS
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
XML_MODELS_DIR = os.path.join(SCRIPT_DIR, "xml_models")
PNG_OUTPUT_DIR = os.path.join(XML_MODELS_DIR, "common")
TEMPLATE_PATH = os.path.join(XML_MODELS_DIR, "3bar_new_platform_all_cables_slope_template.xml")
if not os.path.exists(TEMPLATE_PATH):
    TEMPLATE_PATH = None

png_filename = f"l_ramp_{int(angle_degs)}degs_d{int(d)}m.png"
xml_filename = f"3bar_new_platform_all_cables_ramp_course.xml"

# =============================================================================
# HEIGHTFIELD GENERATION
# =============================================================================
# Grid layout (user coordinate system):
#   - x: left to right,  [0, 4d]
#   - y: top to bottom,  [0, 4d]
#   - Squares indexed 1–4 in each direction (1-based):
#       Square (xi, yi): x in [(xi-1)*d, xi*d], y in [(yi-1)*d, yi*d]
#
# Ramp layout:
#   Square (x=3, y=4): x in [2d, 3d], y in [3d, 4d]
#     → x-direction ramp: height=0 at right edge (x=3d), rises to slope*d at left edge (x=2d)
#   Square (x=2, y=3): x in [d, 2d], y in [2d, 3d]
#     → y-direction ramp: height=slope*d at bottom edge (y=3d), rises to 2*slope*d at top edge (y=2d)
#   All other squares: height=0 (flat ground)

total_size = 4 * d
slope = np.tan(np.radians(angle_degs))
max_height = 2 * slope * d   # peak at top-left corner of square (x=2, y=3)

# Build world-coordinate grids.
# heightfield[row, col] → x = x_coords[col], y = y_coords[row]
# Row 0 = top of user space (y=0), Row H-1 = bottom (y=4d).
x_coords = np.linspace(0, total_size, W)   # shape (W,)
y_coords = np.linspace(0, total_size, H)   # shape (H,)
X, Y = np.meshgrid(x_coords, y_coords)     # both shape (H, W)

heightfield = np.zeros((H, W), dtype=float)

# Square (x=3, y=4) [1-indexed]: x in [2d, 3d], y in [3d, 4d]
# Ramp in x-direction; uniform along y.
mask1 = (X >= 2*d) & (X <= 3*d) & (Y >= 3*d) & (Y <= 4*d)
heightfield[mask1] = slope * (3*d - X[mask1])

# Square (x=2, y=3) [1-indexed]: x in [d, 2d], y in [2d, 3d]
# Ramp in y-direction; picks up where square (3,4) peaked (slope*d at y=3d).
mask2 = (X >= d) & (X <= 2*d) & (Y >= 2*d) & (Y <= 3*d)
heightfield[mask2] = slope * d + slope * (3*d - Y[mask2])

# Normalize to [0, 255]; MuJoCo rescales by HFIELD_Z_MAX.
if max_height > 0:
    heightfield_norm = heightfield / max_height
else:
    heightfield_norm = heightfield
heightfield_uint8 = (255 * heightfield_norm).astype(np.uint8)

# Save PNG
os.makedirs(PNG_OUTPUT_DIR, exist_ok=True)
png_path = os.path.join(PNG_OUTPUT_DIR, png_filename)
Image.fromarray(heightfield_uint8).save(png_path)

print(f"Generated PNG: {png_path}")
print(f"d = {d} m,  angle = {angle_degs} deg")
print(f"Slope (tan): {slope:.6f}")
print(f"Height per square: {slope * d:.4f} m")
print(f"Max height (z_max): {max_height:.4f} m")

# =============================================================================
# GENERATE XML FROM TEMPLATE (Optional)
# =============================================================================
def generate_xml_from_template(template_path, output_path, params):
    """Generate a new XML file from the template with filled parameters."""
    with open(template_path, 'r') as f:
        content = f.read()
    for key, value in params.items():
        placeholder = "{{" + key + "}}"
        content = content.replace(placeholder, str(value))
    with open(output_path, 'w') as f:
        f.write(content)
    print(f"\nGenerated XML: {output_path}")

hfield_x_size = 2 * d   # half-extent in x  (total terrain width  = 4d)
hfield_y_size = 2 * d   # half-extent in y  (total terrain depth  = 4d)

params = {
    "HFIELD_FILE":   f"common/{png_filename}",
    "HFIELD_X_SIZE": hfield_x_size,
    "HFIELD_Y_SIZE": hfield_y_size,
    "HFIELD_Z_MAX":  f"{max_height:.4f}",
    "HFIELD_Z_BASE": hfield_z_base,
}

if TEMPLATE_PATH is not None:
    xml_output_path = os.path.join(XML_MODELS_DIR, xml_filename)
    generate_xml_from_template(TEMPLATE_PATH, xml_output_path, params)
    print(f"\nSummary:")
    print(f"  PNG: {png_path}")
    print(f"  XML: {xml_output_path}")
    print(f"  hfield size: {hfield_x_size} {hfield_y_size} {max_height:.4f} {hfield_z_base}")
else:
    print(f"\nNo template found at {TEMPLATE_PATH}")
    print(f"Only PNG generated: {png_path}")
    print(f"\nRecommended hfield size attribute:")
    print(f'  size="{hfield_x_size} {hfield_y_size} {max_height:.4f} {hfield_z_base}"')

print("\n" + "="*70)
print("To use this heightfield in MuJoCo XML:")
print("="*70)
print(f"""
<asset>
    <hfield name="l_ramp" file="{png_filename}"
            size="{hfield_x_size} {hfield_y_size} {max_height:.4f} {hfield_z_base}"/>
</asset>

<worldbody>
    <geom type="hfield" hfield="l_ramp" pos="0 0 0"/>
</worldbody>
""")

# =============================================================================
# OPTIONAL: 3D VISUALIZATION
# =============================================================================
SHOW_PLOT = False

if SHOW_PLOT:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    Z = heightfield_uint8.astype(float)
    if Z.max() > 0:
        Z = Z / Z.max() * max_height

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(X, Y, Z, cmap="terrain", linewidth=0, antialiased=True)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Height (meters)")
    ax.set_title(f"L-Ramp: angle={angle_degs}°, d={d} m")
    plt.tight_layout()
    plt.show()
