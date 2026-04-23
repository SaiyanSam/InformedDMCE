import cv2
import numpy as np
import sys

def check_map(image_path):
    # Load the image in grayscale, exactly like the C++ logger does
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print(f"Error: Could not load image at {image_path}")
        return

    # Find every unique pixel color value in the image
    unique_values = np.unique(img)
    
    print(f"Successfully loaded map: {image_path}")
    print(f"Image dimensions: {img.shape[1]}x{img.shape[0]} (Width x Height)")
    print("-" * 40)
    print("Unique pixel values found in this map:")
    print(unique_values)
    print("-" * 40)
    print("TYPICAL ROS MAP VALUES:")
    print("  0   = Solid Obstacle / Wall (Black)")
    print("  205 = Unknown / Out of Bounds (Grey)")
    print("  254 or 255 = Free Space / Traversable Corridor (White)")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_map.py <path_to_your_map_image.png_or_pgm>")
    else:
        check_map(sys.argv[1])
