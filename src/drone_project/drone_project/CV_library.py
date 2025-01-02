import numpy as np
import cv2
from scipy.ndimage import convolve


def color_threshold(image, ref_color, threshold, rgb=True):
    """
    Returns a binary image where pixels are 255 if the pixel color is within
    the threshold of the reference color, and 0 elsewhere.

    Parameters:
    - image: Input image (BGR format if rgb=True, grayscale if rgb=False).
    - ref_color: Reference color (tuple). If rgb=True, should be (B, G, R),
                 else a single intensity value.
    - threshold: Threshold value (positive number).
    - rgb: Boolean indicating whether to process in RGB or grayscale mode.

    Returns:
    - binary_image: Binary image with the same dimensions as the input image.
    """
    if rgb:
        # Compute the Euclidean distance in color space
        ref_color_image = np.copy(image)
        ref_color_image[:,:] = ref_color 
        diff = cv2.absdiff(image, ref_color_image)
        distance = np.linalg.norm(diff, axis=2)
        # Create binary mask
        binary_image = np.zeros_like(distance, dtype=np.uint8)
        binary_image[distance <= threshold] = 255
    else:
        # Ensure the image is grayscale
        if len(image.shape) == 3:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray_image = image.copy()
        # Compute absolute difference
        diff = cv2.absdiff(gray_image, np.uint8(ref_color))
        # Create binary mask
        binary_image = np.zeros_like(diff, dtype=np.uint8)
        binary_image[diff <= threshold] = 255

    return binary_image


def find_circles_with_most_pixels(binary_mask, radius, reference_point=None):
    """
    Finds centers of circles of a given radius that include the most 255 pixels,
    excluding pixels in previously found circles.

    Parameters:
    - binary_mask: Binary image (numpy array) with pixel values 0 and 255.
    - radius: Radius of the circles.
    - reference_point: Tuple (y, x) to prefer closer centers. Defaults to image center.

    Returns:
    - centers: List of tuples (x, y) representing the circle centers.
    """
    # Convert binary_mask to 0s and 1s
    binary_mask = (binary_mask == 255).astype(np.uint8)

    # Create circular kernel
    diameter = 2 * radius + 1
    y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
    circular_kernel = (x_indices**2 + y_indices**2 <= radius**2).astype(np.uint8)

    centers = []
    iteration = 0

    while np.any(binary_mask):
        iteration += 1
        print(f"Iteration {iteration}: Remaining pixels = {np.count_nonzero(binary_mask)}")

        # Convolve the binary mask with the circular kernel
        conv_output = convolve(binary_mask, circular_kernel, mode='constant', cval=0)

        # Find the maximum value in the conv_output
        max_value = conv_output.max()

        # Find all positions where the convolution output is equal to max_value
        positions = np.argwhere(conv_output == max_value)

        if positions.size == 0:
            break  # No more positions found

        # Define the reference point (default to center of the image)
        if reference_point is None:
            h, w = binary_mask.shape
            reference_point = np.array([h // 2, w // 2])

        # Compute distances from positions to the reference point
        distances = np.linalg.norm(positions - reference_point, axis=1)

        # Choose the position with minimum distance
        min_index = np.argmin(distances)
        center = positions[min_index]  # (y, x)

        # Add the center to the list of centers
        centers.append((center[1], center[0]))  # (x, y)

        print(f"Selected center at (x={center[1]}, y={center[0]}) covering {max_value} pixels")

        # Exclude the pixels within the circle centered at this point
        # Create a mask with a circle at the selected center
        temp_mask = np.zeros_like(binary_mask)
        cv2.circle(temp_mask, (center[1], center[0]), radius, 1, thickness=-1)  # Note: (x, y)

        # Set the pixels within the circle to 0 in binary_mask
        binary_mask[temp_mask == 1] = 0

    return centers

#the previous function is too expensive in term of computation, we ran convolution every 5 steps

def find_circles_with_most_pixels_optimized(binary_mask, radius, step=5, reference_point=None):
    """
    Optimized version that computes convolution on a grid to find centers of circles
    that include the most 255 pixels, excluding pixels in previously found circles.
    
    Parameters:
    - binary_mask: Binary image (numpy array) with pixel values 0 and 255.
    - radius: Radius of the circles.
    - step: Step size for grid sampling (default is 5).
    - reference_point: Tuple (y, x) to prefer closer centers. Defaults to image center.
    
    Returns:
    - centers: List of tuples (x, y) representing the circle centers.
    """
    # Convert binary_mask to 0s and 1s
    binary_mask = (binary_mask == 255).astype(np.uint8)
    h, w = binary_mask.shape
    
    # Create circular kernel
    diameter = 2 * radius + 1
    y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
    circular_kernel = (x_indices**2 + y_indices**2 <= radius**2).astype(np.uint8)
    
    centers = []
    iteration = 0
    
    while np.any(binary_mask):
        iteration += 1
        print(f"Iteration {iteration}: Remaining pixels = {np.count_nonzero(binary_mask)}")
        
        # Generate grid indices
        y_grid = np.arange(0, h, step)
        x_grid = np.arange(0, w, step)
        
        # Prepare an array to store the convolution results at grid points
        conv_output = np.zeros((len(y_grid), len(x_grid)), dtype=np.int32)
        
        for i, y in enumerate(y_grid):
            for j, x in enumerate(x_grid):
                # Define the region of interest (ROI)
                y_min = max(y - radius, 0)
                y_max = min(y + radius + 1, h)
                x_min = max(x - radius, 0)
                x_max = min(x + radius + 1, w)
                
                roi = binary_mask[y_min:y_max, x_min:x_max]
                
                # Define the corresponding kernel region
                ky_min = radius - (y - y_min)
                ky_max = radius + (y_max - y)
                kx_min = radius - (x - x_min)
                kx_max = radius + (x_max - x)
                
                kernel_roi = circular_kernel[ky_min:ky_max, kx_min:kx_max]
                
                # Compute the sum within the circle at this grid point
                conv_output[i, j] = np.sum(roi * kernel_roi)
        
        # Find the maximum value in the conv_output
        max_value = conv_output.max()
        
        if max_value == 0:
            break  # No more pixels can be covered
        
        # Find all positions where the convolution output is equal to max_value
        positions = np.argwhere(conv_output == max_value)
        
        # Map grid positions back to image coordinates
        positions_in_image = np.array([(y_grid[i], x_grid[j]) for i, j in positions])
        
        # Define the reference point (default to center of the image)
        if reference_point is None:
            reference_point = np.array([h // 2, w // 2])
        
        # Compute distances from positions to the reference point
        distances = np.linalg.norm(positions_in_image - reference_point, axis=1)
        
        # Choose the position with minimum distance
        min_index = np.argmin(distances)
        center = positions_in_image[min_index]  # (y, x)
        
        # Add the center to the list of centers
        centers.append((int(center[1]), int(center[0])))  # (x, y)
        
        print(f"Selected center at (x={center[1]}, y={center[0]}) covering {max_value} pixels")
        
        # Exclude the pixels within the circle centered at this point
        # Create a mask with a circle at the selected center
        temp_mask = np.zeros_like(binary_mask)
        cv2.circle(temp_mask, (int(center[1]), int(center[0])), radius, 1, thickness=-1)  # Note: (x, y)
        
        # Set the pixels within the circle to 0 in binary_mask
        binary_mask[temp_mask == 1] = 0
    
    return centers


#Using agenerator allow us to process in real time the goals instead of waiting for all the procedure
def find_circles_with_most_pixels_generator(binary_mask, radius, step=5, reference_point=None):
    """
    Generator version that computes convolution on a grid to find centers of circles
    that include the most 255 pixels, excluding pixels in previously found circles.
    
    Parameters:
    - binary_mask: Binary image (numpy array) with pixel values 0 and 255.
    - radius: Radius of the circles.
    - step: Step size for grid sampling (default is 5).
    - reference_point: Tuple (y, x) to prefer closer centers. Defaults to image center.
    
    Yields:
    - center: Tuple (x, y) representing the circle center.
    """
    # Convert binary_mask to 0s and 1s
    binary_mask = (binary_mask == 255).astype(np.uint8)
    h, w = binary_mask.shape
    
    # Create circular kernel
    diameter = 2 * radius + 1
    y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
    circular_kernel = (x_indices**2 + y_indices**2 <= radius**2).astype(np.uint8)
    
    iteration = 0
    
    while np.any(binary_mask):
        iteration += 1
        print(f"Iteration {iteration}: Remaining pixels = {np.count_nonzero(binary_mask)}")
        
        # Generate grid indices
        y_grid = np.arange(0, h, step)
        x_grid = np.arange(0, w, step)
        
        # Prepare an array to store the convolution results at grid points
        conv_output = np.zeros((len(y_grid), len(x_grid)), dtype=np.int32)
        
        for i, y in enumerate(y_grid):
            for j, x in enumerate(x_grid):
                # Define the region of interest (ROI)
                y_min = max(y - radius, 0)
                y_max = min(y + radius + 1, h)
                x_min = max(x - radius, 0)
                x_max = min(x + radius + 1, w)
                
                roi = binary_mask[y_min:y_max, x_min:x_max]
                
                # Define the corresponding kernel region
                ky_min = radius - (y - y_min)
                ky_max = radius + (y_max - y)
                kx_min = radius - (x - x_min)
                kx_max = radius + (x_max - x)
                
                kernel_roi = circular_kernel[ky_min:ky_max, kx_min:kx_max]
                
                # Compute the sum within the circle at this grid point
                conv_output[i, j] = np.sum(roi * kernel_roi)
        
        # Find the maximum value in the conv_output
        max_value = conv_output.max()
        
        if max_value == 0:
            break  # No more pixels can be covered
        
        # Find all positions where the convolution output is equal to max_value
        positions = np.argwhere(conv_output == max_value)
        
        # Map grid positions back to image coordinates
        positions_in_image = np.array([(y_grid[i], x_grid[j]) for i, j in positions])
        
        # Define the reference point (default to center of the image)
        if reference_point is None:
            reference_point = np.array([h // 2, w // 2])
        
        # Compute distances from positions to the reference point
        distances = np.linalg.norm(positions_in_image - reference_point, axis=1)
        
        # Choose the position with minimum distance
        min_index = np.argmin(distances)
        center = positions_in_image[min_index]  # (y, x)
        
        print(f"Selected center at (x={center[1]}, y={center[0]}) covering {max_value} pixels")
        
        # Exclude the pixels within the circle centered at this point
        # Create a mask with a circle at the selected center
        temp_mask = np.zeros_like(binary_mask)
        cv2.circle(temp_mask, (int(center[1]), int(center[0])), radius, 1, thickness=-1)  # Note: (x, y)
        
        # Set the pixels within the circle to 0 in binary_mask
        binary_mask[temp_mask == 1] = 0
        
        # Yield the center
        yield (int(center[1]), int(center[0]))


def find_next_circle(binary_mask, radius = 150, step=50, reference_point=None):
    """
    Finds the next center of a circle that includes the most 255 pixels,
    excludes pixels in the circle, and returns the center and updated mask.

    Parameters:
    - binary_mask: Binary image (numpy array) with pixel values 0 and 255.
    - radius: Radius of the circle.
    - step: Step size for grid sampling (default is 5).
    - reference_point: Tuple (y, x) to prefer closer centers. Defaults to image center.

    Returns:
    - center: Tuple (x, y) representing the circle center, or None if no center found.
    - updated_mask: Binary mask with pixels in the circle set to 0.
    """
    # Convert binary_mask to 0s and 1s
    binary_mask = (binary_mask == 255).astype(np.uint8)
    h, w = binary_mask.shape

    if not np.any(binary_mask):
        # No pixels to process
        return None, binary_mask

    # Create circular kernel
    diameter = 2 * radius + 1
    y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
    circular_kernel = (x_indices**2 + y_indices**2 <= radius**2).astype(np.uint8)

    # Generate grid indices
    y_grid = np.arange(0, h, step)
    x_grid = np.arange(0, w, step)

    # Prepare an array to store the convolution results at grid points
    conv_output = np.zeros((len(y_grid), len(x_grid)), dtype=np.int32)

    for i, y in enumerate(y_grid):
        for j, x in enumerate(x_grid):
            # Define the region of interest (ROI)
            y_min = max(y - radius, 0)
            y_max = min(y + radius + 1, h)
            x_min = max(x - radius, 0)
            x_max = min(x + radius + 1, w)

            roi = binary_mask[y_min:y_max, x_min:x_max]

            # Define the corresponding kernel region
            ky_min = radius - (y - y_min)
            ky_max = radius + (y_max - y)
            kx_min = radius - (x - x_min)
            kx_max = radius + (x_max - x)

            kernel_roi = circular_kernel[ky_min:ky_max, kx_min:kx_max]

            # Compute the sum within the circle at this grid point
            conv_output[i, j] = np.sum(roi * kernel_roi)

    # Find the maximum value in the conv_output
    max_value = conv_output.max()

    if max_value == 0:
        # No more pixels can be covered
        return None, binary_mask

    # Find all positions where the convolution output is equal to max_value
    positions = np.argwhere(conv_output == max_value)

    # Map grid positions back to image coordinates
    y_grid_positions = y_grid[positions[:, 0]]
    x_grid_positions = x_grid[positions[:, 1]]
    positions_in_image = np.stack((y_grid_positions, x_grid_positions), axis=-1)

    # Define the reference point (default to center of the image)
    if reference_point is None:
        reference_point = np.array([h // 2, w // 2])

    # Compute distances from positions to the reference point
    distances = np.linalg.norm(positions_in_image - reference_point, axis=1)

    # Choose the position with minimum distance
    min_index = np.argmin(distances)
    center = positions_in_image[min_index]  # (y, x)

    print(f"Selected center at (x={center[1]}, y={center[0]}) covering {max_value} pixels")

    # Exclude the pixels within the circle centered at this point
    # Create a mask with a circle at the selected center
    temp_mask = np.zeros_like(binary_mask)
    cv2.circle(temp_mask, (int(center[1]), int(center[0])), radius, 1, thickness=-1)  # Note: (x, y)

    # Set the pixels within the circle to 0 in binary_mask
    updated_mask = binary_mask.copy()
    updated_mask[temp_mask == 1] = 0

    # Convert center to (x, y) tuple
    center_xy = (int(center[1]), int(center[0]))

    return center_xy, (updated_mask * 255).astype(np.uint8)