import numpy as np
from PIL import Image
import os

# =============================================================================
# CONFIGURATION - Change these values as needed
# =============================================================================
num_steps = 5               # Number of stairs
step_height = 0.5           # Height of each step (meters)
step_depth = 4.0            # Depth of each step (meters)
start_position = 15.0       # Where staircase starts (in meters along x-axis)
x_length = 50.0             # Total terrain length (meters)
W, H = 251, 251             # Heightfield resolution

# Heightfield size parameters (in meters)
hfield_x_size = x_length / 2          # Half-size in X direction
hfield_y_size = 10          # Half-size in Y direction
hfield_z_base = 0.5         # Base elevation

# =============================================================================
# PATHS
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
XML_MODELS_DIR = os.path.join(SCRIPT_DIR, "xml_models")
PNG_OUTPUT_DIR = os.path.join(XML_MODELS_DIR, "common")

# Output filenames (generated based on stairs config)
png_filename = f"staircase_{num_steps}steps_{int(step_height*100)}cm.png"
xml_filename = f"3bar_new_platform_all_cables_staircase_{num_steps}steps.xml"

# Template path (optional - set to None if no template exists)
TEMPLATE_PATH = os.path.join(XML_MODELS_DIR, "3bar_new_platform_all_cables_slope_template.xml")
if not os.path.exists(TEMPLATE_PATH):
    TEMPLATE_PATH = None

# =============================================================================
# HEIGHTFIELD GENERATION
# =============================================================================
total_staircase_height = num_steps * step_height
total_staircase_length = num_steps * step_depth

# X coordinates in meters
x = np.linspace(0, x_length, W)

# Generate staircase profile
height = np.zeros(W)
for i, xi in enumerate(x):
    if xi < start_position:
        # Flat ground before staircase
        height[i] = 0.0
    elif xi < start_position + total_staircase_length:
        # Staircase region
        relative_x = xi - start_position
        step_index = int(relative_x / step_depth)
        step_index = min(step_index, num_steps - 1)  # Cap at last step
        height[i] = step_index * step_height
    else:
        # Flat ground after staircase (at top level)
        height[i] = total_staircase_height

# Expand to 2D (same height across Y axis)
heightfield = np.tile(height, (H, 1))

# Normalize to 0–255 (MuJoCo will scale by z_max in hfield size attribute)
heightfield_normalized = heightfield.copy()
heightfield_normalized -= heightfield_normalized.min()
if heightfield_normalized.max() > 0:
    heightfield_normalized /= heightfield_normalized.max()
heightfield_uint8 = (255 * heightfield_normalized).astype(np.uint8)

# Save PNG
os.makedirs(PNG_OUTPUT_DIR, exist_ok=True)
png_path = os.path.join(PNG_OUTPUT_DIR, png_filename)
Image.fromarray(heightfield_uint8).save(png_path)

print(f"Generated PNG: {png_path}")
print(f"Number of steps: {num_steps}")
print(f"Step height: {step_height} meters")
print(f"Step depth: {step_depth} meters")
print(f"Total staircase height (z_max): {total_staircase_height:.4f} meters")
print(f"Total staircase length: {total_staircase_length:.4f} meters")

# =============================================================================
# GENERATE XML FROM TEMPLATE (Optional)
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

if TEMPLATE_PATH is not None:
    # Template parameters
    params = {
        "HFIELD_FILE": f"common/{png_filename}",
        "HFIELD_X_SIZE": hfield_x_size,
        "HFIELD_Y_SIZE": hfield_y_size,
        "HFIELD_Z_MAX": f"{total_staircase_height:.4f}",
        "HFIELD_Z_BASE": hfield_z_base,
    }

    xml_output_path = os.path.join(XML_MODELS_DIR, xml_filename)
    generate_xml_from_template(TEMPLATE_PATH, xml_output_path, params)

    print(f"\nSummary:")
    print(f"  PNG: {png_path}")
    print(f"  XML: {xml_output_path}")
    print(f"  hfield size: {hfield_x_size} {hfield_y_size} {total_staircase_height:.4f} {hfield_z_base}")
else:
    print(f"\nNo template found at {TEMPLATE_PATH}")
    print(f"Only PNG generated: {png_path}")
    print(f"\nRecommended hfield size attribute:")
    print(f'  size="{hfield_x_size} {hfield_y_size} {total_staircase_height:.4f} {hfield_z_base}"')

# =============================================================================
# OPTIONAL: 2D and 3D VISUALIZATION
# =============================================================================
SHOW_PLOT = False

if SHOW_PLOT:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(14, 5))

    # 2D profile
    ax1 = fig.add_subplot(121)
    ax1.plot(x, height, 'b-', linewidth=2)
    ax1.set_xlabel("X (meters)")
    ax1.set_ylabel("Height (meters)")
    ax1.set_title(f"Staircase Profile: {num_steps} steps")
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, x_length])
    ax1.set_ylim([0, total_staircase_height * 1.2])

    # 3D surface
    ax2 = fig.add_subplot(122, projection="3d")

    # Create world-coordinate grids
    X = np.linspace(0, x_length, W)
    Y = np.linspace(-hfield_y_size, hfield_y_size, H)
    X_mesh, Y_mesh = np.meshgrid(X, Y)

    Z = heightfield_uint8.astype(float)
    Z /= Z.max() if Z.max() > 0 else 1
    Z *= total_staircase_height

    ax2.plot_surface(X_mesh, Y_mesh, Z, cmap="terrain", linewidth=0, antialiased=True, alpha=0.9)
    ax2.set_xlabel("X (meters)")
    ax2.set_ylabel("Y (meters)")
    ax2.set_zlabel("Height (meters)")
    ax2.set_title(f"Staircase: {num_steps} steps × {step_height}m")

    plt.tight_layout()
    plt.show()

print("\n" + "="*70)
print("To use this heightfield in MuJoCo XML:")
print("="*70)
print(f"""
<asset>
    <hfield name="staircase" file="{png_filename}"
            size="{hfield_x_size} {hfield_y_size} {total_staircase_height:.4f} {hfield_z_base}"/>
</asset>

<worldbody>
    <geom type="hfield" hfield="staircase" pos="0 0 0"/>
</worldbody>
""")
