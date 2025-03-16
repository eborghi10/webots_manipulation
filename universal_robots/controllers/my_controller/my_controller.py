from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image

import torch

from controller import Robot

class MyController(Robot):
    def __init__(self):
        super(MyController, self).__init__()
        self.time_step = 32  # set the control time step

        # Get the camera device
        self.camera = self.getDevice("camera")
        self.camera.enable(10)

        # Load Processor & VLA
        self.processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
        self.vla = AutoModelForVision2Seq.from_pretrained(
                "openvla/openvla-v01-7b",
                attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
                torch_dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                trust_remote_code=True,
                force_download=False,
            ).to("cuda:0")

    def run(self):
        # with a parallelized control loop, it may be necessary to run an initial step to initialize sensor values
        self.step(self.time_step)

        prompt = "Grab the can coming to you"

        # main control loop
        while True:
            # begin simulation step computation: send command values to Webots for update
            # leave the loop when the simulation is over
            if self.stepBegin(self.time_step) == -1:
                break

            print("Hello World!")

            # Get Image
            image = self.camera.getImage()
            # TODO: Convert image to PIL Image

            # Predict Action (7-DoF; un-normalize for BridgeV2)
            inputs = self.processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
            action = self.vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

            print(type(action))
            print(action)

            # end simulation step computation: retrieve new sensor values from Webots
            # leave the loop when the simulation is over
            if self.stepEnd() == -1:
                break

# Main program
controller = MyController()
controller.run()
