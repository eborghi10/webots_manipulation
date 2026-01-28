"""
Vision Language Model Controller for Webots Manipulation
A standard Python approach for VLM-based robot control.

Supports multiple VLM backends:
- OpenVLA (openvla/openvla-7b)
- LeRobot policies
- Custom VLMs via transformers

Usage:
    webots universal_robots/worlds/ure.wbt
    python universal_robots/controllers/my_controller/vlm_controller.py
"""

import os
import numpy as np
from PIL import Image
from abc import ABC, abstractmethod
from typing import Optional, Tuple, List
from dataclasses import dataclass

# Webots controller import
from controller import Robot

# Optional: Check for available backends
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. VLM inference disabled.")

try:
    from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False
    print("Warning: Transformers not available.")


@dataclass
class RobotAction:
    """Represents a robot action command."""
    joint_positions: np.ndarray  # 6-DOF joint positions
    gripper_open: float  # 0.0 (closed) to 1.0 (open)
    
    def __post_init__(self):
        assert len(self.joint_positions) == 6, "Expected 6 joint positions"
        assert 0.0 <= self.gripper_open <= 1.0, "Gripper must be in [0, 1]"


class VLMBackend(ABC):
    """Abstract base class for VLM backends."""
    
    @abstractmethod
    def predict_action(self, image: Image.Image, prompt: str) -> RobotAction:
        """Predict robot action from image and language prompt."""
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if this backend is available."""
        pass


class OpenVLABackend(VLMBackend):
    """OpenVLA backend for vision-language-action prediction."""
    
    def __init__(
        self,
        model_name: str = "openvla/openvla-7b",
        device: str = "cuda:0",
        use_flash_attention: bool = False,  # Disabled - flash-attn wheel incompatible with torch 2.10
        load_in_8bit: bool = True,  # Enable 8-bit quantization to reduce memory
    ):
        self.model_name = model_name
        self.device = device
        self.use_flash_attention = use_flash_attention
        self.load_in_8bit = load_in_8bit
        self.processor = None
        self.model = None
        self._loaded = False
        
    def is_available(self) -> bool:
        return TORCH_AVAILABLE and TRANSFORMERS_AVAILABLE
    
    def load(self):
        """Load the model (call this explicitly to control when loading happens)."""
        if self._loaded:
            return
            
        if not self.is_available():
            raise RuntimeError("OpenVLA backend not available. Install torch and transformers.")
        
        print(f"Loading OpenVLA model: {self.model_name}...")
        
        self.processor = AutoProcessor.from_pretrained(
            self.model_name, 
            trust_remote_code=True,
            use_fast=False
        )
        
        model_kwargs = {
            "low_cpu_mem_usage": True,
            "trust_remote_code": True,
            "attn_implementation": "eager",  # Disable SDPA - OpenVLA doesn't support it
        }
        
        if self.load_in_8bit:
            quantization_config = BitsAndBytesConfig(load_in_8bit=True)
            model_kwargs["quantization_config"] = quantization_config
            model_kwargs["device_map"] = "auto"
        else:
            model_kwargs["torch_dtype"] = torch.bfloat16
        
        if self.use_flash_attention:
            model_kwargs["attn_implementation"] = "flash_attention_2"
        
        self.model = AutoModelForVision2Seq.from_pretrained(
            self.model_name,
            **model_kwargs
        )
        
        # Only move to device if not using quantization (device_map handles it)
        if not self.load_in_8bit:
            self.model = self.model.to(self.device)
        
        self._loaded = True
        print("OpenVLA model loaded successfully!")
    
    def predict_action(self, image: Image.Image, prompt: str) -> RobotAction:
        """Predict action from image and prompt."""
        if not self._loaded:
            self.load()
        
        # Ensure RGB format
        if image.mode != "RGB":
            image = image.convert("RGB")
        
        # Process inputs - don't force dtype when using quantization
        inputs = self.processor(prompt, image).to(self.device)
        
        # Predict action (7-DoF: 6 joints + gripper)
        # Using bridge_orig unnorm_key - adjust based on your robot
        action = self.model.predict_action(
            **inputs, 
            unnorm_key="bridge_orig",  # Change this for different robots
            do_sample=False
        )
        
        # Convert to numpy and split into joints + gripper
        action_np = action.cpu().numpy() if isinstance(action, torch.Tensor) else np.array(action)
        
        # OpenVLA outputs 7 values: x, y, z, roll, pitch, yaw, gripper
        # Gripper: -1 = close, +1 = open, normalize to [0, 1]
        gripper_raw = float(action_np[6]) if len(action_np) > 6 else 0.0
        gripper_normalized = (gripper_raw + 1.0) / 2.0  # Map [-1,1] to [0,1]
        gripper_normalized = max(0.0, min(1.0, gripper_normalized))  # Clamp
        
        return RobotAction(
            joint_positions=action_np[:6],
            gripper_open=gripper_normalized
        )


class MockVLMBackend(VLMBackend):
    """Mock backend for testing without GPU/model."""
    
    def __init__(self):
        self.step_count = 0
    
    def is_available(self) -> bool:
        return True
    
    def predict_action(self, image: Image.Image, prompt: str) -> RobotAction:
        """Return a simple oscillating action for testing."""
        self.step_count += 1
        
        # Simple oscillating motion for testing
        t = self.step_count * 0.05
        return RobotAction(
            joint_positions=np.array([
                np.sin(t) * 0.1,      # shoulder_pan
                -1.0 + np.sin(t) * 0.1,  # shoulder_lift
                1.5,                   # elbow
                -1.5,                  # wrist_1
                -1.57,                 # wrist_2
                0.0,                   # wrist_3
            ]),
            gripper_open=0.5 + np.sin(t * 2) * 0.3
        )


class VLMController(Robot):
    """
    Webots controller with Vision Language Model integration.
    
    This controller captures camera images, processes them with a VLM,
    and executes the predicted actions on the robot.
    """
    
    # Joint names for UR5e
    JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint", 
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    
    # Gripper finger names for Robotiq 3F
    GRIPPER_FINGERS = [
        "finger_1_joint_1",
        "finger_2_joint_1",
        "finger_middle_joint_1",
    ]
    
    def __init__(
        self,
        vlm_backend: Optional[VLMBackend] = None,
        time_step: int = 32,
        use_mock: bool = False,
    ):
        super().__init__()
        self.time_step = time_step
        
        # Initialize VLM backend
        if vlm_backend is not None:
            self.vlm = vlm_backend
        elif use_mock or not TORCH_AVAILABLE:
            print("Using mock VLM backend for testing")
            self.vlm = MockVLMBackend()
        else:
            self.vlm = OpenVLABackend()
        
        # Initialize camera
        self.camera = self.getDevice("camera")
        if self.camera:
            self.camera.enable(self.time_step)
            self.camera_width = self.camera.getWidth()
            self.camera_height = self.camera.getHeight()
            print(f"Camera initialized: {self.camera_width}x{self.camera_height}")
        else:
            print("Warning: No camera found!")
            self.camera_width = 0
            self.camera_height = 0
        
        # Initialize joint motors
        self.motors = {}
        self.position_sensors = {}
        for name in self.JOINT_NAMES:
            motor = self.getDevice(name)
            if motor:
                self.motors[name] = motor
                # Get position sensor
                sensor = motor.getPositionSensor()
                if sensor:
                    sensor.enable(self.time_step)
                    self.position_sensors[name] = sensor
        
        # Initialize gripper motors
        self.gripper_motors = {}
        for name in self.GRIPPER_FINGERS:
            motor = self.getDevice(name)
            if motor:
                self.gripper_motors[name] = motor
        
        print(f"Initialized {len(self.motors)} arm joints, {len(self.gripper_motors)} gripper joints")
    
    def get_camera_image(self) -> Optional[Image.Image]:
        """Capture and convert Webots camera image to PIL Image."""
        if not self.camera:
            return None
        
        # Get raw image data from Webots (BGRA format)
        image_data = self.camera.getImage()
        if image_data is None:
            return None
        
        # Convert to numpy array
        image_array = np.frombuffer(image_data, dtype=np.uint8)
        image_array = image_array.reshape((self.camera_height, self.camera_width, 4))
        
        # Convert BGRA to RGB
        rgb_array = image_array[:, :, [2, 1, 0]]  # Swap B and R channels
        
        # Convert to PIL Image
        return Image.fromarray(rgb_array, mode="RGB")
    
    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        positions = []
        for name in self.JOINT_NAMES:
            if name in self.position_sensors:
                positions.append(self.position_sensors[name].getValue())
            else:
                positions.append(0.0)
        return np.array(positions)
    
    def set_joint_positions(self, positions: np.ndarray):
        """Set target joint positions."""
        for i, name in enumerate(self.JOINT_NAMES):
            if name in self.motors and i < len(positions):
                self.motors[name].setPosition(float(positions[i]))
    
    def set_gripper(self, openness: float):
        """
        Set gripper openness.
        openness: 0.0 = closed, 1.0 = open
        """
        # Robotiq 3F gripper: 0 = open, ~1.2 = closed
        target_position = (1.0 - openness) * 1.2
        
        for motor in self.gripper_motors.values():
            motor.setPosition(target_position)
    
    def execute_action(self, action: RobotAction, current_positions: np.ndarray):
        """
        Execute a robot action.
        
        OpenVLA outputs delta actions (relative movements), so we add them
        to current positions. The first 6 values are typically:
        - For joint control: delta joint positions
        - For task space: dx, dy, dz, droll, dpitch, dyaw
        
        We scale the deltas since OpenVLA outputs are typically small.
        """
        # Scale factor for delta actions (adjust based on robot/simulation)
        delta_scale = 10.0  # Amplify the small deltas from OpenVLA
        
        # Apply delta to current joint positions
        new_positions = current_positions + action.joint_positions * delta_scale
        
        self.set_joint_positions(new_positions)
        self.set_gripper(action.gripper_open)
    
    def run(self, prompt: str = "Pick up the can"):
        """
        Main control loop.
        
        Args:
            prompt: Natural language instruction for the task
        """
        print(f"Starting VLM control with prompt: '{prompt}'")
        
        # Initial step to initialize sensors
        self.step(self.time_step)
        
        # Load VLM model
        if isinstance(self.vlm, OpenVLABackend):
            self.vlm.load()
        
        step_count = 0
        
        while True:
            # Begin simulation step
            if self.stepBegin(self.time_step) == -1:
                break
            
            # Capture camera image
            image = self.get_camera_image()
            if image is None:
                print("Warning: No camera image available")
                if self.stepEnd() == -1:
                    break
                continue
            
            # Get current state (for logging/debugging)
            current_positions = self.get_joint_positions()
            
            # Predict action using VLM
            try:
                action = self.vlm.predict_action(image, prompt)
                
                # Log every 10 steps
                if step_count % 10 == 0:
                    print(f"Step {step_count}: delta = {action.joint_positions[:3]}... gripper={action.gripper_open:.2f}")
                
                # Execute the action (apply deltas to current positions)
                self.execute_action(action, current_positions)
                
            except Exception as e:
                print(f"Error in VLM prediction: {e}")
            
            step_count += 1
            
            # End simulation step
            if self.stepEnd() == -1:
                break
        
        print(f"Control loop ended after {step_count} steps")


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="VLM Controller for Webots")
    parser.add_argument(
        "--prompt", 
        type=str, 
        default="Pick up the can coming towards you",
        help="Natural language instruction"
    )
    parser.add_argument(
        "--mock", 
        action="store_true",
        help="Use mock VLM for testing without GPU"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="openvla/openvla-7b",
        help="VLM model to use"
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Device for model inference"
    )
    
    args = parser.parse_args()
    
    # Create backend
    if args.mock:
        backend = MockVLMBackend()
    else:
        backend = OpenVLABackend(
            model_name=args.model,
            device=args.device,
        )
    
    # Create and run controller
    controller = VLMController(vlm_backend=backend)
    controller.run(prompt=args.prompt)


if __name__ == "__main__":
    main()
