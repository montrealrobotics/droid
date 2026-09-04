import numpy as np
import cv2

DEFAULT_MAX_FORCE_CAPACITY = 1200.0  # Max reading from sensor
DEFAULT_TACTILE_IMAGE_SIZE = (32, 32)


def normalize_tactile_frame(
    ts_frame: np.ndarray,
    max_capacity: float = DEFAULT_MAX_FORCE_CAPACITY
) -> np.ndarray:
    """
    Applies scaling [0.0, 1.0].
    """
    frame = ts_frame.astype(np.float32)

    return np.clip(frame / max_capacity, 0.0, 1.0)


def tactile_frame_to_image(
    norm_tactile_frame: np.ndarray,
    target_size: tuple[int, int] = DEFAULT_TACTILE_IMAGE_SIZE
) -> np.ndarray:
    """
    Converts single or set of tactile frames to image.

    Args:
        norm_tactile_frame: Normalized float array

    Returns:
        np.ndarray: 1-channel float image of shape (H, W, 1) in range [0.0, 1.0]
    """
    num_fingers = norm_tactile_frame.shape[0]
    f0 = norm_tactile_frame[0]
    f1 = norm_tactile_frame[1] if num_fingers > 1 else np.zeros_like(f0)

    combined_grid = np.hstack([f0, f1])

    # Resample using OpenCV (expects uint8 for INTER_CUBIC)
    uint8_grid = (combined_grid * 255.0).astype(np.uint8)
    resized_img = cv2.resize(uint8_grid, dsize=target_size, interpolation=cv2.INTER_CUBIC)

    return (resized_img.astype(np.float32) / 255.0)[..., np.newaxis]


def tactile_image_stack(
    subframe_list: list[np.ndarray],
    max_capacity: float = DEFAULT_MAX_FORCE_CAPACITY,
    target_size: tuple[int, int] = DEFAULT_TACTILE_IMAGE_SIZE
) -> np.ndarray:
    """
    Takes a list of frames
    normalizes them, converts to image, stacks them.
    """
    image_list = []
    for raw_f in subframe_list:
        norm_f = normalize_tactile_frame(raw_f, max_capacity=max_capacity)
        img_f = tactile_frame_to_image(norm_f, target_size=target_size)
        image_list.append(img_f)

    return np.stack(image_list, axis=0)


def average_tactile_subframes(
    subframe_list: list[np.ndarray],
    max_capacity: float = DEFAULT_MAX_FORCE_CAPACITY,
    target_size: tuple[int, int] = DEFAULT_TACTILE_IMAGE_SIZE
) -> np.ndarray:
    """
    Takes a list of frames
    normalizes them, converts to image, stacks and averages them into a single step image.
    """

    return np.mean(tactile_image_stack(subframe_list), axis=0)