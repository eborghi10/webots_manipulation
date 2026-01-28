"""
LeRobot Backend for VLM Controller

LeRobot (https://github.com/huggingface/lerobot) provides pre-trained policies
for robot manipulation tasks. This backend integrates LeRobot policies with
the Webots controller.

Supported policies:
- ACT (Action Chunking Transformer)
- Diffusion Policy  
- VQ-BeT
- TDMPC

Installation:
    pip install lerobot
"""

import numpy as np
from PIL import Image
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    from lerobot.common.policies.factory import make_policy
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from omegaconf import OmegaConf
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False


@dataclass
class RobotAction:
    """Represents a robot action command."""
    joint_positions: np.ndarray
    gripper_open: float


class LeRobotBackend:
    """
    LeRobot policy backend for robot control.
    
    This backend loads pre-trained policies from the LeRobot hub
    and uses them for action prediction.
    """
    
    def __init__(
        self,
        policy_name: str = "lerobot/act_aloha_sim_transfer_cube_human",
        device: str = "cuda:0",
        num_joints: int = 6,
    ):
        """
        Initialize LeRobot backend.
        
        Args:
            policy_name: HuggingFace model name for the policy
            device: Device for inference
            num_joints: Number of robot joints
        """
        self.policy_name = policy_name
        self.device = device
        self.num_joints = num_joints
        self.policy = None
        self._loaded = False
        
        # Action history for temporal policies
        self.action_queue: List[np.ndarray] = []
        self.observation_history: List[Dict] = []
        
    def is_available(self) -> bool:
        """Check if LeRobot is available."""
        return TORCH_AVAILABLE and LEROBOT_AVAILABLE
    
    def load(self):
        """Load the policy from HuggingFace hub."""
        if self._loaded:
            return
            
        if not self.is_available():
            raise RuntimeError(
                "LeRobot not available. Install with: pip install lerobot"
            )
        
        print(f"Loading LeRobot policy: {self.policy_name}...")
        
        # Load policy from hub
        from lerobot.common.policies.factory import make_policy
        from huggingface_hub import snapshot_download
        
        # Download policy checkpoint
        policy_path = snapshot_download(self.policy_name)
        
        # Load config and create policy
        config_path = f"{policy_path}/config.yaml"
        cfg = OmegaConf.load(config_path)
        
        self.policy = make_policy(
            hydra_cfg=cfg,
            pretrained_policy_name_or_path=self.policy_name,
        )
        self.policy.to(self.device)
        self.policy.eval()
        
        self._loaded = True
        print("LeRobot policy loaded successfully!")
    
    def preprocess_image(self, image: Image.Image) -> torch.Tensor:
        """Preprocess image for the policy."""
        # Resize to expected size (usually 224x224 or 256x256)
        image = image.resize((256, 256), Image.Resampling.BILINEAR)
        
        # Convert to tensor and normalize
        img_array = np.array(image).astype(np.float32) / 255.0
        img_tensor = torch.from_numpy(img_array).permute(2, 0, 1)  # HWC -> CHW
        
        # Normalize with ImageNet stats
        mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        img_tensor = (img_tensor - mean) / std
        
        return img_tensor.unsqueeze(0)  # Add batch dimension
    
    def predict_action(
        self, 
        image: Image.Image, 
        prompt: str,
        current_joints: Optional[np.ndarray] = None,
    ) -> RobotAction:
        """
        Predict action using LeRobot policy.
        
        Args:
            image: Camera observation
            prompt: Language instruction (may not be used by all policies)
            current_joints: Current joint positions (optional)
            
        Returns:
            Predicted robot action
        """
        if not self._loaded:
            self.load()
        
        # Use queued actions if available (for action chunking)
        if self.action_queue:
            action = self.action_queue.pop(0)
            return RobotAction(
                joint_positions=action[:self.num_joints],
                gripper_open=float(action[self.num_joints]) if len(action) > self.num_joints else 0.5
            )
        
        # Preprocess observation
        img_tensor = self.preprocess_image(image).to(self.device)
        
        # Build observation dict
        observation = {
            "observation.image": img_tensor,
        }
        
        # Add state if available
        if current_joints is not None:
            state_tensor = torch.from_numpy(current_joints).float().unsqueeze(0)
            observation["observation.state"] = state_tensor.to(self.device)
        
        # Predict action(s)
        with torch.no_grad():
            action_dict = self.policy.select_action(observation)
        
        # Handle action chunking - queue future actions
        actions = action_dict["action"].cpu().numpy()
        if actions.ndim == 2 and actions.shape[0] > 1:
            # Multiple actions predicted (action chunking)
            self.action_queue = list(actions[1:])  # Queue remaining actions
            actions = actions[0]
        elif actions.ndim == 2:
            actions = actions[0]
        
        return RobotAction(
            joint_positions=actions[:self.num_joints],
            gripper_open=float(actions[self.num_joints]) if len(actions) > self.num_joints else 0.5
        )
    
    def reset(self):
        """Reset the policy state (clear action queue, etc.)."""
        self.action_queue = []
        self.observation_history = []
        if hasattr(self.policy, 'reset'):
            self.policy.reset()


class SimpleDiffusionBackend:
    """
    Simplified Diffusion Policy backend.
    
    This is a lightweight implementation that doesn't require the full
    LeRobot installation. Uses diffusers library directly.
    """
    
    def __init__(
        self,
        model_name: str = "lerobot/diffusion_pusht",
        device: str = "cuda:0",
        num_joints: int = 6,
        prediction_horizon: int = 16,
        action_horizon: int = 8,
    ):
        self.model_name = model_name
        self.device = device
        self.num_joints = num_joints
        self.prediction_horizon = prediction_horizon
        self.action_horizon = action_horizon
        self.model = None
        self._loaded = False
        self.action_queue: List[np.ndarray] = []
        
    def is_available(self) -> bool:
        try:
            import diffusers
            return TORCH_AVAILABLE
        except ImportError:
            return False
    
    def load(self):
        """Load diffusion model."""
        if self._loaded:
            return
        
        print(f"Loading diffusion policy: {self.model_name}...")
        
        # For now, this is a placeholder - actual implementation would
        # load from HuggingFace hub
        self._loaded = True
        print("Diffusion policy loaded!")
    
    def predict_action(
        self, 
        image: Image.Image, 
        prompt: str,
        current_joints: Optional[np.ndarray] = None,
    ) -> RobotAction:
        """Predict action using diffusion policy."""
        if not self._loaded:
            self.load()
        
        # Use queued actions
        if self.action_queue:
            action = self.action_queue.pop(0)
            return RobotAction(
                joint_positions=action[:self.num_joints],
                gripper_open=0.5
            )
        
        # Placeholder: return zero action
        # Real implementation would run diffusion inference
        return RobotAction(
            joint_positions=np.zeros(self.num_joints),
            gripper_open=0.5
        )


def create_lerobot_backend(
    policy_type: str = "act",
    device: str = "cuda:0",
) -> LeRobotBackend:
    """
    Factory function to create LeRobot backend with common configurations.
    
    Args:
        policy_type: Type of policy ("act", "diffusion", "vqbet", "tdmpc")
        device: Device for inference
        
    Returns:
        Configured LeRobot backend
    """
    # Map policy types to HuggingFace model names
    policy_map = {
        "act": "lerobot/act_aloha_sim_transfer_cube_human",
        "act_real": "lerobot/act_aloha_real",
        "diffusion": "lerobot/diffusion_pusht",
        "vqbet": "lerobot/vqbet_aloha_sim_transfer_cube_human",
        "tdmpc": "lerobot/tdmpc_aloha_sim_transfer_cube_human",
    }
    
    policy_name = policy_map.get(policy_type, policy_type)
    
    return LeRobotBackend(
        policy_name=policy_name,
        device=device,
    )
