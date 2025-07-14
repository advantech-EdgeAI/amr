import rclpy
import numpy as np
import PIL.Image
import time
import yaml
from threading import Thread
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
# from nano_llm import NanoLLM, ChatHistory
from vllm import LLM, SamplingParams
from dataclasses import dataclass
from pathlib import Path



class CaptionerNode(Node):

    def __init__(self, args):
        super().__init__("CaptionerNode")

        self.declare_parameter("model", "deepseek-ai/deepseek-vl2-tiny")
        self.declare_parameter("image_topic", "/front_stereo_camera/left/image_raw")
        # self.declare_parameter("image_topic", "/webcam_image")
        self.declare_parameter("caption_topic", "/caption")
        self.declare_parameter("caption_ready_topic", "/caption_ready")
        self.declare_parameter("caption_pose_topic", "/caption_pose")
        self.declare_parameter(
            "prompt",
            "<|user|>\n<image>\n<image>\n<image>\n<image>\n<image>\n<image>\n" + \
            "Describe in detail what is happening throughout the sequence of six images. Do not describe each image separately. Avoid descriptive embellishments." + \
            "Specifically focus on the people, objects, furniture, environmental features, events/ectivities, signs, use of each room, and other interesting details." + \
            "<|end|>\n<|assistant|>\n"
        )
        # self.declare_parameter(
        #     "prompt",
        #     "<video> This video shows a clock. Please describe what time is it."
        # )
        self.declare_parameter("video_name_topic", "/video_name")

        self.declare_parameter("use_every_nth_image", 10)
        self.declare_parameter("caption_image_count", 6)
        self.declare_parameter("caption_interval", 3.0)

        try:
            data = {}
            config_path = Path(args.config_path)
            if config_path.exists():
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
                    data = config.get(self.get_name(), {})
            for param in data:
                if self.has_parameter(param):
                    self.set_parameters([Parameter(param, self.get_parameter(param).type_, data[param])])
        except Exception:# as e:
            pass
            
        # print("image topic: ", self.get_parameter("image_topic").value)

        self.image_subscriber = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.image_callback,
            1
        )

        self.caption_publisher = self.create_publisher(
            String, 
            self.get_parameter("caption_topic").value,
            10
        )

        self.caption_ready = True
        self.caption_ready_publisher = self.create_publisher(
            Bool, 
            self.get_parameter("caption_ready_topic").value,
            1
        )
        
        self.caption_pose_publisher = self.create_publisher(
            Bool, 
            self.get_parameter("caption_pose_topic").value,
            1
        )
        
        self.caption_subscriber = self.create_subscription(
            String,
            self.get_parameter("video_name_topic").value,
            self.clean_buffer_callback,
            10
        )

        self.debug = False
        self.cv_bridge = CvBridge()
        if not self.debug:
            # llm = LLM(model="google/gemma-3-4b-it")  # Replace with Gemma 3B when available
            self.llm = LLM(model="deepseek-ai/deepseek-vl2-tiny",
                           limit_mm_per_prompt={"image": 6},
                           hf_overrides={"architectures": ["DeepseekVLV2ForCausalLM"]})
            self.sampling_params = SamplingParams(temperature=0.1, max_tokens=512)


        self.prompt = self.get_parameter("prompt").value.strip("][()")
        self.use_every_nth_image = self.get_parameter("use_every_nth_image").value
        self.caption_interval = self.get_parameter("caption_interval").value
        self.caption_topic = self.get_parameter("caption_topic").value
        self.caption_image_count = self.get_parameter("caption_image_count").value
        self.caption_interval = self.get_parameter("caption_interval").value

        # state
        self.image_buffer = []
        self.image_counter = 0

        self.caption_loop_thread = None
        self.caption_loop_running = False
        self.logger = self.get_logger()
        self.logger.info(f"image topic: {self.get_parameter('image_topic').value}")
        self.logger.info(f"Caption Node is ready.")

    def start_caption_loop(self):

        self.caption_loop_running = True
        thread = Thread(target=self.caption_loop)
        thread.start()
        self.caption_loop_thread = thread

    def stop_caption_loop(self):

        self.caption_loop_running = False
        self.caption_loop_thread.join()
        self.caption_loop_thread = None

    def caption_loop(self):

        last_publish = time.perf_counter()

        while self.caption_loop_running:
            
            dt = time.perf_counter() - last_publish

            if dt < self.caption_interval:
                time.sleep(self.caption_interval - dt)

            # get last N images
            images = [b for b in self.image_buffer]
            
            if len(images) < self.caption_image_count:
                self.logger.info("Skipped image captioning for current time window.  No images available in buffer.")
            else:
                self.publish_caption_pose(True)
                self.logger.info(f"Start caption.")
                caption = self.caption_images(images)
                self.logger.info(f"Generated caption using {len(images)} images.")
                self.publish_caption(caption)
                self.logger.info(f"Published caption: " + caption)

            last_publish = time.perf_counter()
            
    def image_callback(self, image_msg: Image):
        
        if self.image_counter % self.use_every_nth_image == 0:
            self.logger.info(f"Received image. {self.image_counter}")
            np_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'rgb8')
            pil_image = PIL.Image.fromarray(np_image)
            timestamp = time.perf_counter()
            self.image_buffer.append(pil_image)
            if len(self.image_buffer) > self.caption_image_count:
                self.image_buffer = self.image_buffer[1:]
            
        self.image_counter += 1
    
    def caption_images(self, images):

        self.caption_ready = False
        caption_ready_msg = Bool()
        caption_ready_msg.data = self.caption_ready
        self.caption_ready_publisher.publish(caption_ready_msg)

        if self.debug:
            return "Dummy caption"
        
        start = time.time()

        # Generate output
        outputs = self.llm.generate(
            [
                {
                    "prompt": self.prompt,
                    "multi_modal_data": {"image": images},
                }
            ],
            self.sampling_params
        )

        # Clean up
        for o in outputs:
            caption = o.outputs[0].text
        print("***Caption uses ", time.time()-start)

        self.caption_ready = True
        caption_ready_msg = Bool()
        caption_ready_msg.data = self.caption_ready
        self.caption_ready_publisher.publish(caption_ready_msg)
        
        return caption
    
    def publish_caption(self, caption: str):

        caption_msg = String()
        caption_msg.data = caption
        self.caption_publisher.publish(caption_msg)

    def publish_caption_pose(self, caption: bool):
        
        caption_pose_msg = Bool()
        caption_pose_msg.data = True
        self.caption_pose_publisher.publish(caption_pose_msg)

    def clean_buffer_callback(self, msg: String):
        while not self.caption_ready:
            time.sleep(1)

        self.image_buffer = []
        self.image_counter = 0

def main(args=None):
    rclpy.init()
    node = CaptionerNode(args)
    node.start_caption_loop()
    rclpy.spin(node)
    node.stop_caption_loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()