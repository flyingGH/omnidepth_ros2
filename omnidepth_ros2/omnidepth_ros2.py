import onnxruntime as ort
import numpy as np
import rclpy
import cv2
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
import threading
import time
from .utils.math_utils import rotate_around_point_highperf
import torch
from typing import List, Tuple


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

MONO3D_NAMES = ['car', 'truck', 'bus', 
                'trailer', 'construction_vehicle',
                'pedestrian', 'motorcycle', 'bicycle',
                'traffic_cone', 'barrier']



def normalize_image(image):
    rgb_mean = np.array([0.485, 0.456, 0.406])
    rgb_std  = np.array([0.229, 0.224, 0.225])
    image = image.astype(np.float32)
    image = image / 255.0
    image = image - rgb_mean
    image = image / rgb_std
    return image


class BaseInferenceThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.build_model(*args, **kwargs)
    
    def build_model(self, onnx_path, gpu_index=0):
        providers = [("CUDAExecutionProvider", {"cudnn_conv_use_max_workspace": '0', 'device_id': str(gpu_index)})]
        sess_options = ort.SessionOptions()
        self.ort_session = ort.InferenceSession(onnx_path, providers=providers, sess_options=sess_options)
        input_shape = self.ort_session.get_inputs()[0].shape # [1, 3, h, w]
        self.inference_h = input_shape[2]
        self.inference_w = input_shape[3]
    
    def set_inputs(self, image, P=None):
        self.image = image
        self.P = P

    def resize(self, image, P=None):
        self.h0, self.w0 = image.shape[0:2]
        scale = min(self.inference_h / self.h0, self.inference_w / self.w0)
        self.scale = scale
        self.h_eff = int(self.h0 * scale)
        self.w_eff = int(self.w0 * scale)
        final_image = np.zeros([self.inference_h, self.inference_w, 3])
        final_image[0:self.h_eff, 0:self.w_eff] = cv2.resize(image, (self.w_eff, self.h_eff),
                                                    interpolation=cv2.INTER_LINEAR)
        if P is None:
            return final_image
        else:
            P = P.copy()
            P[0:2, :] = P[0:2, :] * scale
            return final_image, P

    def deresize(self, seg):
        seg = seg[0:self.h_eff, 0:self.w_eff]
        seg = cv2.resize(seg, (self.w0, self.h0), interpolation=cv2.INTER_NEAREST)
        return seg

    def run(self):
        raise NotImplementedError

    def join(self):
        threading.Thread.join(self)
        threading.Thread.__init__(self)
        return self._output


class Metric3DThread(BaseInferenceThread):
    def run(self):
        start_time = time.time()
        # Prepare input for model inference
        onnx_input, pad_info = self.prepare_input(self.image)
        # Perform inference
        outputs = self.ort_session.run(None, onnx_input)
        depth_image = outputs[0][0, 0] # [1, 1, H, W] -> [H, W]
        point_cloud = outputs[1] # [HW, 6]
        mask = outputs[2] # [HW]
        # print(point_cloud.shape, mask.shape)
        point_cloud = point_cloud[mask] # [HW, 6]
        point_cloud = point_cloud.reshape([-1, 6])
        
        depth_image = depth_image[pad_info[0] : depth_image.shape[0] - pad_info[1], pad_info[2] : depth_image.shape[1] - pad_info[3]] # [H, W] -> [h, w]     
        self._output = depth_image, point_cloud
        print(f"monodepth runtime: {time.time() - start_time}")

    def prepare_input(self, rgb_image: np.ndarray)->Tuple[torch.Tensor, List[int]]:
        input_size = (616, 1064)

        h, w = rgb_image.shape[:2]
        scale = min(input_size[0] / h, input_size[1] / w)
        self.scale = scale
        rgb = cv2.resize(rgb_image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)

        padding = [123.675, 116.28, 103.53]
        h, w = rgb.shape[:2]
        pad_h = input_size[0] - h
        pad_w = input_size[1] - w
        pad_h_half = pad_h // 2
        pad_w_half = pad_w // 2
        rgb:np.ndarray = cv2.copyMakeBorder(rgb, pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half, cv2.BORDER_CONSTANT, value=padding)
        pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]

        P = self.P.copy()
        P[0:2, :] = P[0:2, :] * scale
        
        # Create P_inv
        P_expanded = np.eye(4)
        P_expanded[0:3, 0:4] = P
        P_inv = np.linalg.inv(P_expanded) # 4x4

        # Create T
        T = np.eye(4)

        # Create mask
        H, W = input_size
        mask = np.zeros([H, W], dtype=np.uint8)
        mask[pad_info[0] : H - pad_info[1], pad_info[2] : W - pad_info[3]] = 1

        onnx_input = {
            'image': np.ascontiguousarray(np.transpose(rgb, (2, 0, 1))[None], dtype=np.float32) , # 1, 3, H, W
            'P': P.astype(np.float32)[None], # 1, 3, 4
            'P_inv': P_inv.astype(np.float32)[None], # 1, 4, 4
            'T': T.astype(np.float32)[None], # 1, 4, 4
            'mask' : mask.astype(bool)[None] # 1, H, W
        }
        return onnx_input, pad_info
    

class VisionInferenceNode():
    def __init__(self):
        self.ros_interface = ROSInterface("VisionInferenceNode")

        self.logger = self.ros_interface.get_logger()
        self.clock = self.ros_interface.get_clock()
        self._read_params()
        self._init_model()
        self._init_static_memory()
        self._init_topics()

        self.logger.info("Initialization Done")
        self.ros_interface.spin()
    

    def _read_params(self):
        self.logger.info("Reading parameters...")
        self.monodepth_flag = self.ros_interface.read_one_parameters("MONODEPTH_FLAG", True)
        self.cam_channel_name = self.ros_interface.read_one_parameters("CAM_CHANNEL_NAME")
        
        if self.monodepth_flag:
            self.monodepth_weight_path = self.ros_interface.read_one_parameters("MONODEPTH_CKPT_FILE",
                                        "src/omnidepth_ros2/models/metric_3d.onnx")
            self.monodepth_gpu_index   = int(self.ros_interface.read_one_parameters("MONODEPTH_GPU_INDEX", 0))
        self.gpu_index = int(self.ros_interface.read_one_parameters("GPU", 0))
        

    def _init_model(self):
        self.logger.info("Initializing model...")
        if self.monodepth_flag:
            self.monodepth_thread = Metric3DThread(self.monodepth_weight_path, gpu_index=self.monodepth_gpu_index)
        self.logger.info("Model Done")
    
    def _init_static_memory(self):
        self.logger.info("Initializing static memory...")
        self.frame_id = None
        self.P = None
        self.num_objects = 0
    

    def _init_topics(self):
        self.ros_interface.create_publisher(Image, "depth_image", 10)
        self.ros_interface.create_publisher(PointCloud2, "point_cloud", 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ros_interface.create_subscription(CameraInfo, "/camera_info", self.camera_info_callback, qos_profile=qos_profile)
        self.ros_interface.create_subscription(Image, "/image_raw", self.camera_callback, 1)
        self.ros_interface.create_subscription(CompressedImage, "/compressed_image", self.compressed_image_callback, qos_profile=qos_profile)
        self.ros_interface.clear_all_bbox()


    def camera_info_callback(self, msg:CameraInfo):
        self.P = np.zeros((3, 4))
        self.P[0:3, 0:3] = np.array(msg.k.reshape((3, 3)))
        self.frame_id = msg.header.frame_id

    def process_image(self, image:np.ndarray):        
        starting = time.time()
        if self.monodepth_flag:
            self.monodepth_thread.set_inputs(image, self.P.copy())
            self.monodepth_thread.start()

        depth = self.monodepth_thread.join() if self.monodepth_flag else None

        self.logger.info(f"Total runtime: {time.time() - starting}")

        # publish depth        
        if self.monodepth_flag:

            self.ros_interface.publish_image(depth[0], image_topic="depth_image", frame_id=self.frame_id)
                        
            rotated_pcl = depth[1]
            
            cam_channel_name_to_rotation_dict = {
                "left": 1.5708,
                "front": 0, 
                "right": -1.5708, 
                "back": 3.14159 
            }
            
            num_pts = rotated_pcl.shape[0]
            for p_idx in range(num_pts):
                rotated_pcl[p_idx][2], rotated_pcl[p_idx][0] = rotate_around_point_highperf((rotated_pcl[p_idx][2], rotated_pcl[p_idx][0]),
                                                                                           radians=cam_channel_name_to_rotation_dict[self.cam_channel_name])
            
            pcl_without_gpl = []
            for p_idx in range(num_pts):
                MAX_PCL_HEIGHT = -3.0
                MIN_PCL_HEIGHT = 0.3
                if rotated_pcl[p_idx][1] > MAX_PCL_HEIGHT and rotated_pcl[p_idx][1] < MIN_PCL_HEIGHT:
                    pcl_without_gpl.append(rotated_pcl[p_idx])
            
            pcl_without_gpl = np.asarray(pcl_without_gpl) # Shape: num_pts, 6
            
            self.logger.info(str(pcl_without_gpl.shape))
            
            pcl_without_gpl[:, [3, 5]] = pcl_without_gpl[:, [5, 3]]
            
            self.ros_interface.publish_point_cloud(pcl_without_gpl, "point_cloud", frame_id="base_link", field_names='xyzrgb')
            
            return


    def compressed_image_callback(self, msg:CompressedImage):
        if self.P is None:
            self.logger.info("Waiting for camera info...", throttle_duration_sec=0.5)
            return # wait for camera info
        image = self.ros_interface.cv_bridge.compressed_imgmsg_to_cv2(msg)[..., ::-1]
        self.process_image(image)

    def camera_callback(self, msg:Image):
        if self.P is None:
            self.logger.info("Waiting for camera info...", throttle_duration_sec=0.5)
            return # wait for camera info
        height = msg.height
        width  = msg.width
        
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))[:, :, ::-1]
        self.process_image(image)

def main(args=None):
    rclpy.init(args=args)
    VisionInferenceNode()

if __name__ == "__main__":
    main()