# Webots Manipulator with Vision Language Models

A Webots simulation for robot manipulation using Vision Language Models (VLMs) to control a Universal Robot arm.

## Features

- **OpenVLA Integration**: Use OpenVLA-7B for vision-language-action prediction
- **LeRobot Support**: Optional integration with HuggingFace LeRobot policies (ACT, Diffusion Policy, etc.)
- **Mock Mode**: Test without GPU using mock backend
- **UR5e Robot**: Universal Robots UR5e with Robotiq 3F gripper
- **Camera Integration**: Real-time camera image processing

## Getting Started

### Prerequisites

1. [Webots R2025a](https://cyberbotics.com/) or later
2. Python 3.10+
3. CUDA-capable GPU (for VLM inference) or use mock mode

### Installation

```bash
# Install Python dependencies
cd universal_robots/controllers/my_controller
pip install -r requirements.txt

# For best OpenVLA performance, install flash-attention
pip install flash-attn --no-build-isolation
```

### World Setup

The UR5e robot is configured with an external controller. If you need to reconfigure:

1. Open `universal_robots/worlds/ure.wbt`
2. Find the UR5e node and set `controller` to `<extern>`
3. Save and restart the simulation

Reference: https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers

## Usage

### Run with OpenVLA (requires GPU)

```bash
# Terminal 1: Start Webots
webots universal_robots/worlds/ure.wbt

# Terminal 2: Run VLM controller
python3 universal_robots/controllers/my_controller/vlm_controller.py \
    --prompt "Pick up the can coming towards you"
```

### Run in Mock Mode (no GPU required)

```bash
# Test the controller without VLM inference
python3 universal_robots/controllers/my_controller/vlm_controller.py --mock
```

### Custom Model

```bash
python3 universal_robots/controllers/my_controller/vlm_controller.py \
    --model "openvla/openvla-7b" \
    --device "cuda:0" \
    --prompt "Grasp the red object"
```

## Controllers

| File | Description |
|------|-------------|
| `vlm_controller.py` | **Recommended**: Clean VLM controller with OpenVLA backend |
| `lerobot_backend.py` | LeRobot policy backend (ACT, Diffusion, etc.) |
| `my_controller.py` | Original OpenVLA prototype |
| `dora_controller.py` | Dora-rs based controller (RDT-1B) |

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│     Webots      │────▶│  VLM Controller │────▶│   VLM Backend   │
│   Simulation    │     │                 │     │  (OpenVLA/etc)  │
│                 │◀────│  - Camera       │◀────│                 │
│   UR5e Robot    │     │  - Joints       │     │  Action Predict │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

## Supported VLM Backends

### OpenVLA (Default)
- Model: `openvla/openvla-7b`
- Best for: General manipulation tasks
- Requirements: ~16GB VRAM with bfloat16

### LeRobot (Optional)
- Policies: ACT, Diffusion Policy, VQ-BeT, TDMPC
- Best for: Task-specific fine-tuned models
- Install: `pip install lerobot`

## Troubleshooting

### Camera not working
Ensure the camera is enabled in the world file and the `enable()` call uses a valid time step.

### Out of memory
Try reducing model precision or use a smaller model:
```python
backend = OpenVLABackend(device="cuda:0", use_flash_attention=True)
```

### No GPU available
Use mock mode for testing: `--mock`

## References

- [OpenVLA](https://github.com/openvla/openvla)
- [LeRobot](https://github.com/huggingface/lerobot)
- [Webots Documentation](https://cyberbotics.com/doc/reference/index)
- [Unified Video Action Model](https://unified-video-action-model.github.io/)
