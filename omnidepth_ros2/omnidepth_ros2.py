import onnxruntime as ort
import numpy as np
import rclpy
import cv2
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
from visualization_msgs.msg import MarkerArray
import threading
from numba import jit
from .seg_labels import PALETTE
import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

MONO3D_NAMES = ['car', 'truck', 'bus', 
                'trailer', 'construction_vehicle',
                'pedestrian', 'motorcycle', 'bicycle',
                'traffic_cone', 'barrier']

COLOR_MAPPINGS = {
    'car' : (  0,  0,142),  'truck': (  0,  0, 70) ,
    'bus': (  0, 60,100), 'trailer': (  0,  0,110),
    'construction_vehicle':  (  0,  0, 70), 'pedestrian': (220, 20, 60),
    'motorcycle': (  0,  0,230), 'bicycle': (119, 11, 32),
    'traffic_cone': (180,165,180), 'barrier': (190,153,153)
}

@jit(nopython=True, cache=True)
def ColorizeSeg(pred_seg, rgb_image, opacity=1.0, palette=PALETTE):
    color_seg = np.zeros((pred_seg.shape[0], pred_seg.shape[1], 3), dtype=np.uint8)
    h, w = pred_seg.shape
    for i in range(h):
        for j in range(w):
            color_seg[i, j] = palette[pred_seg[i, j]]
    new_image = rgb_image * (1 - opacity) + color_seg * opacity
    new_image = new_image.astype(np.uint8)
    return new_image

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

class Mono3D(BaseInferenceThread):
    def run(self):
        global MONO3D_NAMES
        start_time = time.time()
        resized_image, resized_P = self.resize(self.image, self.P)
        input_numpy = np.ascontiguousarray(np.transpose(normalize_image(resized_image), (2, 0, 1))[None], dtype=np.float32)
        P_numpy = np.array(resized_P, dtype=np.float32)[None]
        outputs = self.ort_session.run(None, {'image': input_numpy, 'P2': P_numpy})
        scores = np.array(outputs[0], dtype=np.float64) # N
        bboxes = np.array(outputs[1], dtype=np.float64) # N, 12
        cls_indexes = outputs[2] # N

        cls_names = [MONO3D_NAMES[cls_index] for cls_index in cls_indexes]

        objects = []
        N = len(bboxes)
        
        
        for i in range(N):
            obj = {}
            obj['whl'] = bboxes[i, 7:10]
            obj['theta'] = bboxes[i, 11]
            obj['score'] = scores[i]
            obj['type_name'] = cls_names[i]
            obj['xyz'] = bboxes[i, 4:7]
            objects.append(obj)
        self._output = objects
        print(f"mono3d runtime: {time.time() - start_time}")

class SegmentationThread(BaseInferenceThread):
    def run(self):
        start_time = time.time()
        resized_image = self.resize(self.image)
        input_numpy = np.ascontiguousarray(np.transpose(normalize_image(resized_image), (2, 0, 1))[None], dtype=np.float32)
        outputs = self.ort_session.run(None, {'input': input_numpy})
        self._output = self.deresize(np.array(outputs[0][0], np.uint8))
        print(f"segmentation runtime: {time.time() - start_time}")

class MonodepthThread(BaseInferenceThread):
    def run(self):
        start_time = time.time()
        resized_image, resized_P = self.resize(self.image, self.P)
        input_numpy = np.ascontiguousarray(np.transpose(normalize_image(resized_image), (2, 0, 1))[None], dtype=np.float32)
        P_numpy = np.array(resized_P, dtype=np.float32)[None]
        outputs = self.ort_session.run(None, {'image': input_numpy, 'P2': P_numpy})
        self._output = self.deresize(outputs[0][0, 0])
        print(f"monodepth runtime: {time.time() - start_time}")


import torch
from typing import List, Tuple
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

        self.mono3d_flag = self.ros_interface.read_one_parameters("MONO3D_FLAG", True)
        self.seg_flag = self.ros_interface.read_one_parameters("SEG_FLAG", True)
        self.monodepth_flag = self.ros_interface.read_one_parameters("MONODEPTH_FLAG", True)
        self.cam_channel_name = self.ros_interface.read_one_parameters("CAM_CHANNEL_NAME")
        
        
        if self.mono3d_flag:
            self.mono3d_weight_path  = self.ros_interface.read_one_parameters("MONO3D_CKPT_FILE",
                                    "/home/yliuhb/vision_factory/weights/mono3d.onnx")
            self.mono3d_gpu_index    = int(self.ros_interface.read_one_parameters("MONO3D_GPU_INDEX", 0))

        if self.seg_flag:
            self.seg_weight_path = self.ros_interface.read_one_parameters("SEG_CKPT_FILE",
                                        "/home/yliuhb/vision_factory/weights/seg.onnx")
            self.seg_gpu_index   = int(self.ros_interface.read_one_parameters("SEG_GPU_INDEX", 0))
        
        if self.monodepth_flag:
            self.monodepth_weight_path = self.ros_interface.read_one_parameters("MONODEPTH_CKPT_FILE",
                                        "/home/yliuhb/vision_factory/weights/monodepth.onnx")
            self.monodepth_gpu_index   = int(self.ros_interface.read_one_parameters("MONODEPTH_GPU_INDEX", 0))
            self.monodepth_use_metric_3d = self.ros_interface.read_one_parameters("MONODEPTH_USE_METRIC_3D", False)
        
        self.gpu_index = int(self.ros_interface.read_one_parameters("GPU", 0))
        self.seg_opacity = float(self.ros_interface.read_one_parameters("opacity", 0.9))

    def _init_model(self):
        self.logger.info("Initializing model...")
        if self.mono3d_flag:
            self.mono3d_thread = Mono3D(self.mono3d_weight_path, gpu_index=self.mono3d_gpu_index)
        if self.seg_flag:
            self.seg_thread    = SegmentationThread(self.seg_weight_path, gpu_index=self.seg_gpu_index)
        if self.monodepth_flag:
            if self.monodepth_use_metric_3d:
                self.monodepth_thread = Metric3DThread(self.monodepth_weight_path, gpu_index=self.monodepth_gpu_index)
            else:
                self.monodepth_thread = MonodepthThread(self.monodepth_weight_path, gpu_index=self.monodepth_gpu_index)
        self.logger.info("Model Done")
    
    def _init_static_memory(self):
        self.logger.info("Initializing static memory...")
        self.frame_id = None
        self.P = None
        self.num_objects = 0
    

    def _init_topics(self):
        self.bbox_publish = self.ros_interface.create_publisher(MarkerArray, "mono3d/bbox", 10)
        self.ros_interface.create_publisher(Image, "seg_image", 10)
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
        if self.mono3d_flag:
            self.mono3d_thread.set_inputs(image, self.P.copy())
            self.mono3d_thread.start()
        if self.seg_flag:
            self.seg_thread.set_inputs(image)
            self.seg_thread.start()
        if self.monodepth_flag:
            self.monodepth_thread.set_inputs(image, self.P.copy())
            self.monodepth_thread.start()


        objects = self.mono3d_thread.join() if self.mono3d_flag else None
        seg = self.seg_thread.join() if self.seg_flag else None
        depth = self.monodepth_thread.join() if self.monodepth_flag else None

        self.logger.info(f"Total runtime: {time.time() - starting}")

        # publish objects
        if self.mono3d_flag:
            marker_array = MarkerArray()
            for i, obj in enumerate(objects):
                class_name = obj['type_name']
                marker = self.ros_interface.object_to_marker(obj,
                                                            self.frame_id,
                                                            i,
                                                            color=COLOR_MAPPINGS[class_name],
                                                            duration=0.5) # color maps
                marker_array.markers.append(marker)
            self.bbox_publish.publish(marker_array)

        # publish colorized seg
        if self.seg_flag:
            seg_image = ColorizeSeg(seg, image, opacity=self.seg_opacity)
            self.ros_interface.publish_image(seg_image[:, :, ::-1], image_topic="seg_image", frame_id=self.frame_id)

        # publish depth
        if self.monodepth_flag and not self.monodepth_use_metric_3d:
            self.ros_interface.publish_image(depth, image_topic="depth_image", frame_id=self.frame_id)
        
        if self.monodepth_flag and self.monodepth_use_metric_3d:

            self.ros_interface.publish_image(depth[0], image_topic="depth_image", frame_id=self.frame_id)
            
            
            import math
            
            def rotate_around_point_lowperf(point, radians, origin=(0, 0)):
                """Rotate a point around a given point.
                
                Source: https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302
                TODO: High perf. algorithm also present.
                
                I call this the "low performance" version since it's recalculating
                the same values more than once [cos(radians), sin(radians), x-ox, y-oy).
                It's more readable than the next function, though.
                """
                x, y = point
                ox, oy = origin

                qx = ox + math.cos(radians) * (x - ox) + math.sin(radians) * (y - oy)
                qy = oy + -math.sin(radians) * (x - ox) + math.cos(radians) * (y - oy)

                return qx, qy
            
            def rotate_around_point_highperf(xy, radians, origin=(0, 0)):
                """Rotate a point around a given point.

                Source: https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302

                
                I call this the "high performance" version since we're caching some
                values that are needed >1 time. It's less readable than the previous
                function but it's faster.
                """
                x, y = xy
                offset_x, offset_y = origin
                adjusted_x = (x - offset_x)
                adjusted_y = (y - offset_y)
                cos_rad = math.cos(radians)
                sin_rad = math.sin(radians)
                qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
                qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

                return qx, qy
            
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
            
            # Below wwas original.
            # self.ros_interface.publish_point_cloud(depth[1], "point_cloud", frame_id=self.frame_id, field_names='xyzrgb')
            return

        # publish colorized point cloud
        if self.monodepth_flag:
            point_cloud = self.ros_interface.depth_image_to_point_cloud_array(depth,
                                                                              self.P[0:3, 0:3],
                                                                              rgb_image=seg_image if self.seg_flag else image)
            mask = (point_cloud[:, 1] > -3.5) * (point_cloud[:, 1] < 5.5) * (point_cloud[:, 2] < 80)
            point_cloud = point_cloud[mask]
                    
            self.ros_interface.publish_point_cloud(point_cloud, "point_cloud", frame_id=self.frame_id, field_names='xyzrgb')

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
