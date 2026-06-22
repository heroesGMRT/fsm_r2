"""Vision interface for mission_fsm."""


class VisionInterface:
    """Interface for vision-related actions."""

    def detect(self, target):
        """Perform a vision detection task."""
        raise NotImplementedError
