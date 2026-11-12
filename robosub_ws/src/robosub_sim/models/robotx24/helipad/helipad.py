import cv2
import numpy as np
from PIL import Image

# constants
px_per_in = 50
square_in = 80
size = square_in * px_per_in
circle_thickness = int(0.5 * px_per_in)

# Create image using numpy array for cv2
img = np.full((size, size, 3), [211, 211, 211], dtype=np.uint8)  # lightgray RGB

# circle diameters
d_mid = 24 * px_per_in
d_large = 60 * px_per_in

cx, cy = size // 2, size // 2

# draw circles using cv2 for crisp edges
for d in [d_large, d_mid]:
    r = d // 2
    cv2.circle(img, (cx, cy), r, (0, 0, 0), circle_thickness, lineType=cv2.LINE_AA)

# load logo
logo_pil = Image.open("robonation logo-black-icon_cropped.png").convert("RGBA")
max_logo = 12 * px_per_in
logo_ratio = min(max_logo / logo_pil.width, max_logo / logo_pil.height)
logo_resized = logo_pil.resize(
    (int(logo_pil.width * logo_ratio), int(logo_pil.height * logo_ratio))
)

# Convert logo to numpy and overlay on image
logo_np = np.array(logo_resized)
lx = cx - logo_np.shape[1] // 2
ly = cy - logo_np.shape[0] // 2

# Blend logo using alpha channel
for c in range(3):
    img[ly : ly + logo_np.shape[0], lx : lx + logo_np.shape[1], c] = (
        logo_np[:, :, c] * (logo_np[:, :, 3] / 255.0)
        + img[ly : ly + logo_np.shape[0], lx : lx + logo_np.shape[1], c]
        * (1 - logo_np[:, :, 3] / 255.0)
    ).astype(np.uint8)

# save
out_path = "helipad_big_logo.png"
cv2.imwrite(out_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
