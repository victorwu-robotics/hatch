"""
Base Camera Driver - Abstract interface for all depth-sensing devices.

Defines the contract that all camera and scanner drivers must fulfill.
Uses Python's abc module — no Qt dependency.

Principle: Pure Python. No Qt in driver abstractions.
"""

from abc import ABC, abstractmethod
from typing import Optional, Tuple
import numpy as np


class BaseCameraDriver(ABC):
    """
    Abstract base class for all depth-sensing devices.

    Subclasses implement hardware-specific capture logic.
    The interface is the same whether the device is an RGB-D camera
    (Orbbec, RealSense) or a laser scanner producing point clouds
    (Keyence LJ-V7200).

    Each subclass must implement:
    - start_streaming()
    - capture_raw_pointcloud()
    - stop_streaming()
    """

    def __init__(self, device_sn: Optional[str] = None):
        """
        Initialize the camera driver.

        Args:
            device_sn: Optional device serial number for multi-camera setups.
        """
        self.device_sn = device_sn
        self.is_streaming = False

    @abstractmethod
    def start_streaming(self):
        """
        Initialize device and begin data acquisition.

        Must set self.is_streaming = True on success.
        Called once before capture_raw_pointcloud().
        """
        ...

    @abstractmethod
    def capture_raw_pointcloud(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Capture one frame from the device.

        Returns:
            points: (N, 3) float32 array of XYZ coordinates in the device's
                    native frame (typically the optical frame). Units: meters.
                    Returns None if no frame is available.
            colors: (N, 3) uint8 array of RGB values (0-255).
                    Returns None if no frame is available.
        """
        ...

    @abstractmethod
    def stop_streaming(self):
        """
        Stop data acquisition and release device resources.

        Must set self.is_streaming = False.
        """
        ...