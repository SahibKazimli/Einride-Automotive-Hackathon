"""Pure helpers for validating perception message timestamps."""


def is_detection_stamp_fresh(offset_seconds: float,
                              max_offset_seconds: float = 1.0) -> bool:
    """Return whether a detection stamp is close enough to the current ROS time.

    `offset_seconds` is now minus the detection stamp. Positive values are old
    detections; negative values are stamps from the future.
    """
    return abs(offset_seconds) <= max_offset_seconds
