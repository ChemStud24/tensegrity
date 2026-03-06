import numpy as np
from PIL import Image
import os

# =============================================================================
# CONFIGURATION - Change these values as needed
# =============================================================================
angle_degs = 10          # Slope angle in degrees
start, end = 20, 30      # Where slope starts and ends (in meters along x-axis)
x_length = 50.0          # Total terrain length (meters)
W, H = 256, 256          # Heightfield resolution

# Heightfield size parameters (in meters)
hfield_x_size = 50       # Half-size in X direction
hfield_y_size = 20       # Half-size in Y direction
hfield_z_base = 0.5      # Base elevation

# =============================================================================
# PATHS
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
XML_MODELS_DIR = os.path.join(SCRIPT_DIR, "gnn_simulator/mujoco_physics_engine/xml_models")
PNG_OUTPUT_DIR = os.path.join(XML_MODELS_DIR, "common")
TEMPLATE_PATH = os.path.join(XML_MODELS_DIR, "3bar_new_platform_all_cables_slope_template.xml")

# Output filenames (generated based on angle)
png_filename = f"slope_{angle_degs}degs.png"
xml_filename = f"3bar_new_platform_all_cables_slope_{angle_degs}degs.xml"

# =============================================================================
# HEIGHTFIELD GENERATION
# =============================================================================
slope_angle = angle_degs * np.pi / 180
slope_height = np.tan(slope_angle) * (end - start)

# X coordinates
x = np.linspace(0, x_length, W)

height = np.zeros(W)
for i, xi in enumerate(x):
    if xi < start:
        height[i] = 0.0
    elif xi < end:
        height[i] = np.tan(slope_angle) * (xi - start)
    else:
        height[i] = slope_height

# Expand to 2D
heightfield = np.tile(height, (H, 1))

# Normalize to 0–255 (MuJoCo will scale by z_max in hfield size attribute)
heightfield -= heightfield.min()
if heightfield.max() > 0:
    heightfield /= heightfield.max()
heightfield = (255 * heightfield).astype(np.uint8)

# Save PNG
os.makedirs(PNG_OUTPUT_DIR, exist_ok=True)
png_path = os.path.join(PNG_OUTPUT_DIR, png_filename)
Image.fromarray(heightfield).save(png_path)

print(f"Generated PNG: {png_path}")
print(f"Slope angle: {angle_degs} degrees")
print(f"Slope height (z_max): {slope_height:.4f} meters")

# =============================================================================
# GENERATE XML FROM TEMPLATE
# =============================================================================
def generate_xml_from_template(template_path, output_path, params):
    """Generate a new XML file from the template with filled parameters."""
    with open(template_path, 'r') as f:
        content = f.read()
    
    # Replace all placeholders
    for key, value in params.items():
        placeholder = "{{" + key + "}}"
        content = content.replace(placeholder, str(value))
    
    with open(output_path, 'w') as f:
        f.write(content)
    
    print(f"\nGenerated XML: {output_path}")

# Template parameters
params = {
    "HFIELD_FILE": f"common/{png_filename}",
    "HFIELD_X_SIZE": hfield_x_size,
    "HFIELD_Y_SIZE": hfield_y_size,
    "HFIELD_Z_MAX": f"{slope_height:.4f}",
    "HFIELD_Z_BASE": hfield_z_base,
}

xml_output_path = os.path.join(XML_MODELS_DIR, xml_filename)
generate_xml_from_template(TEMPLATE_PATH, xml_output_path, params)

print(f"\nSummary:")
print(f"  PNG: {png_path}")
print(f"  XML: {xml_output_path}")
print(f"  hfield size: {hfield_x_size} {hfield_y_size} {slope_height:.4f} {hfield_z_base}")

# =============================================================================
# OPTIONAL: 3D VISUALIZATION
# =============================================================================
SHOW_PLOT = False

if SHOW_PLOT:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Create world-coordinate grids
    X = np.linspace(0, x_length, W)
    Y = np.linspace(-5, 5, H)
    X, Y = np.meshgrid(X, Y)

    Z = heightfield.astype(float)
    Z /= Z.max()
    Z *= slope_height

    fig = plt.figure(figsize=(10, 4))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot_surface(X, Y, Z, cmap="terrain", linewidth=0, antialiased=True)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Height (meters)")
    ax.set_title(f"Slope: {angle_degs} degrees")

    plt.tight_layout()
    plt.show()
