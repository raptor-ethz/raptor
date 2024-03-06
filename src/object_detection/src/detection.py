#!/usr/bin/env python3
from detectron2.utils.logger import setup_logger


# Initalize logger from detectron2
setup_logger()

import numpy as np
import math
import cv2
import time

# detectron2 dependencies
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.video_visualizer import VideoVisualizer
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog

# Local dependencies
from pointcloud import GraspCandidate
# import zmq
import utils
from logger import Logger
from frame_transformations import transform_frame_EulerXYZ
from pointcloud import GraspCandidate
# import detection_msg_pb2
from streamer_receiver import VideoReceiver
from message_filters import ApproximateTimeSynchronizer, Subscriber

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from raptor_interface.msg import Detection, Pose

# Different runtime options

# Show window with detection visualisation
SHOW_WINDOW_VIS = True

# Send output to external target application using ZMQ 
# If this is enabled and the target application isn't connected, this application won't work
SEND_OUTPUT = False

# Use a naive localization method for the object centroid, just picking the center of the pointcloud
SIMPLE_LOC = True

# Filtering of the grasp that is sent to the external target application
SEND_RAW = False
SEND_MEAN = False
SEND_ROLLING_AVG = True

# Debugging option to record point cloud data from different points
RECORD_PCD_DATA = False

# Target object the system will publish coordinates for
TARGET_OBJECT = 'person'

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.starting_time = time.time()
        self.color_sub = Subscriber(
            self,
            CompressedImage,
            '/color_compressed')
        self.depth_sub = Subscriber(
            self,
            CompressedImage,
            '/depth_compressed')
        self.publisher = self.create_publisher(Detection, 'detection', 10)
        # The receiver for the image frames sent over the local network
        # self.receiver = VideoReceiver()
        self.cam = utils.RSCameraMockup()

        self.output = cv2.VideoWriter(utils.VIDEO_FILE, cv2.VideoWriter_fourcc(
                'M', 'J', 'P', 'G'), 10, (self.cam.width, self.cam.height))

        self.output_depth = cv2.VideoWriter(utils.VIDEO_DEPTH_FILE, cv2.VideoWriter_fourcc(
            'M', 'J', 'P', 'G'), 10, (self.cam.width, self.cam.height))

        self.output_raw = cv2.VideoWriter(utils.VIDEO_RAW_FILE, cv2.VideoWriter_fourcc(
            'M', 'J', 'P', 'G'), 10, (self.cam.width, self.cam.height))

        self.output_grasp = cv2.VideoWriter(utils.VIDEO_GRASP_FILE, cv2.VideoWriter_fourcc(
            'M', 'J', 'P', 'G'), 10, (self.cam.width, self.cam.height))


        # detectron2 setup - change as needed
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml')
        self.predictor = DefaultPredictor(cfg)

        self.bridge = CvBridge()

        metadata = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])
        self.v = VideoVisualizer(metadata)

        # Setup for a custom logger
        self.logger = Logger()
        self.records = np.empty((0, self.logger.cols))
        self.time_logger = Logger()

        # Holds transit times of frames for debugging
        self.times = []

        # Initialize the ZMQ socket for connecting to an external target application
        # context = zmq.Context()
        # socket = context.socket(zmq.REP)

        # if SEND_OUTPUT:
        #     socket.connect('tcp://localhost:2222')
        self.grasp = GraspCandidate()

        self.starting_time = time.time()
        self.frame_counter = 0

        # Counts frames in which the target object was found
        self.success_frame_counter = 0
        self.elapsed_time = 0

        # serial_msg is the message that will be sent to the external target application
        # quad_pose is the pose we will receive from the external target application to perform frame transformations
        self.serial_msg = None
        self.quad_pose = Detection()

        # self.quad_sub = self.create_subscription(Pose, 'px4_pose_nwu', self.quad_callback, 10)

        # This line is particularly important to get the class labels
        self.class_catalog = metadata.thing_classes

        ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1, allow_headerless=True)
        # ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.quad_sub], queue_size=10, slop=0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
    
    # def quad_callback(self, quad_pose):
    #     self.quad_pose.x = quad_pose.x_m
    #     self.quad_pose.y = quad_pose.y_m
    #     self.quad_pose.z = quad_pose.z_m
    #     self.quad_pose.roll = quad_pose.roll_deg
    #     self.quad_pose.pitch = quad_pose.pitch_deg
    #     self.quad_pose.yaw = quad_pose.yaw_deg
        # print("Quad message received")

    def callback(self, frame, depth_frame):
        # Receive the frames from the ARM computer
        # frame, depth_frame = self.receiver.recv_frames()
        serial_msg = None
  
        cam_intrinsics = self.cam.intrinsics

        # print("Image message received")

        # frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding='passthrough')
        frame_arr = np.frombuffer(frame.data, np.uint8)
        frame = cv2.imdecode(frame_arr, cv2.IMREAD_COLOR)
        # frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)  

        # depth_frame = self.bridge.imgmsg_to_cv2(depth_frame, desired_encoding='passthrough')
        depth_arr = np.frombuffer(depth_frame.data, np.uint8)
        depth_frame = cv2.imdecode(depth_arr, cv2.IMREAD_ANYDEPTH)

        # Write the RGB frame to the output without annotations        
        # self.output_raw.write(frame)
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # print(frame.shape, depth_frame.shape)
        # print(depth_frame.dtype)  # Should be np.uint8, np.uint16 or np.float32

        # Get detectron2 output
        outputs = self.predictor(frame)
        detected_class_idxs = outputs['instances'].pred_classes
        pred_boxes = outputs['instances'].pred_boxes
        
        # Show visualization if enabled
        if SHOW_WINDOW_VIS:
            out = self.v.draw_instance_predictions(frame, outputs['instances'].to('cpu'))
            vis_frame = np.asarray(out.get_image())

        # This holds the masks generated by detectron2
        mask_array = outputs['instances'].pred_masks.to('cpu').numpy()
        mask_array = np.moveaxis(mask_array, 0, -1)
        num_instances = mask_array.shape[-1]
        mask_array_instance = []
        
        # Loop over instances that have been detected
        for i in range(num_instances):
            class_idx = detected_class_idxs[i]
            class_name = self.class_catalog[class_idx]
            
            # Get bounding box if needed
            bbox = pred_boxes[i].to('cpu')
            [(xmin, ymin, xmax, ymax)] = bbox
            # cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)
            

            # Naive localization based on bounding box center
            if SIMPLE_LOC:
                xmin = int(xmin)
                ymin = int(ymin)
                xmax = int(xmax)
                ymax = int(ymax)

                center_x = (xmax - xmin)//2 + xmin
                center_y = (ymax - ymin)//2 + ymin

                if center_y < self.cam.height and center_x < self.cam.width:
                            depth = depth_frame[center_y,
                                center_x].astype(float)
                            distance = depth * self.cam.depth_scale
                else:
                    # No valid distance found
                    distance = 0.0
                    print('invalid coordinates')


                # Get translation vector relative to the camera frame
                tvec = self.cam.deproject(cam_intrinsics, center_x, center_y, distance)
                easy_center = tvec
                
                # Realsense y-axis points down by default
                tvec[1] = -tvec[1]
                yaw = 0

            # Create the mask frame that can be applied on the RGB image
            mask_array_instance.append(mask_array[:, :, i:(i+1)])
            obj_mask = np.zeros_like(frame)
            obj_mask = np.where(mask_array_instance[i] == True, 255, obj_mask)
            # cv2.imwrite(f'pictures/mask_{class_name}.png',obj_mask)

            # Target has been detected - apply frame transformation and/or send coordinates
            if class_name == TARGET_OBJECT:
                
                # Simple localization - take naive estimate from bounding box
                if SEND_OUTPUT and SIMPLE_LOC:
                    print('Object location (simple localization) -----')
                      
                    # Holds the translation from the camera frame to the drone frame
                    cam_2_drone_translation = [0.1267, -0.01, 0.0]
                    # Holds the rotation from the camera frame to the drone frame
                    cam_2_drone_orientation = [0, -30, 0]

                    # Holds the translation from the drone frame to the global frame
                    translation = [
                        self.quad_pose.x, self.quad_pose.y, self.quad_pose.z]
                    # Holds the rotation from the drone frame to the global frame
                    rotation = [
                        self.quad_pose.roll, -self.quad_pose.pitch, self.quad_pose.yaw]

                    # Align axes accordingly to motion capture system
                    # Turn into 4-vector for homogenous coordinates
                    tvec = [tvec[2], tvec[0], tvec[1], 1]

                    
                    # Transform into drone frame
                    tvec = transform_frame_EulerXYZ(cam_2_drone_orientation, cam_2_drone_translation, tvec, degrees=True)

                    # Transform into mocap frame
                    tvec = transform_frame_EulerXYZ(
                        rotation, translation, tvec, degrees=False)
                    
                    
                # Create Protobuf message to send object location
                msg = Detection()

                
                # Create point cloud of detected object

                # First, apply mask to RGB frame
                masked_frame = cv2.bitwise_and(frame, obj_mask)
                print("MASK:",masked_frame.dtype)
                # cv2.imwrite('masked_frame.png', masked_frame)
                
                # Debugging
                if RECORD_PCD_DATA:
                    grasp_color = GraspCandidate()
                    grasp_masked = GraspCandidate()
                    
                    grasp_masked.set_point_cloud_from_aligned_masked_frames(masked_frame, depth_frame, cam_intrinsics)
                    grasp_color.set_point_cloud_from_aligned_frames(frame, depth_frame, cam_intrinsics)
                    
                    grasp_masked.save_pcd(f'pcd/pcd_logs/{utils.RECORD_COUNTER}_{TARGET_OBJECT}_masked_{frame_counter}.pcd')
                    grasp_color.save_pcd(f'pcd/pcd_logs/{utils.RECORD_COUNTER}_{TARGET_OBJECT}_full_{frame_counter}.pcd')
                    
                    print(f'Recorded pcd for frame {frame_counter}, sleeping briefly')
                    time.sleep(3)
                
                
                grasp_points = None
                try:
                    # Create point cloud 
                    self.grasp.set_point_cloud_from_aligned_masked_frames(masked_frame, depth_frame, cam_intrinsics)
                    centroid = self.grasp.find_centroid()
                    axis_ext, _, _ = self.grasp.find_largest_axis()
                    # Get longest axis
                    axis = axis_ext[0]
                    # Rotate copy of point cloud around longest axis
                    pcd = self.grasp.rotate_pcd_around_axis(self.grasp.pointcloud, centroid, math.pi, axis)
                    self.grasp.pointcloud += pcd
                    self.grasp.save_pcd(f'pcd/pointcloud_{TARGET_OBJECT}_{utils.RECORD_COUNTER}.pcd')
                
                    # Find grasping points
                    grasp_points = self.grasp.find_grasping_points()
                    
                except Exception as e:
                    # Sometimes, qhull fails and throws an error and we don't want the program to crash
                    print('pcd data analysis went wrong')
                    print(e)
                

                # Yaw computation needs to be adapted to current way of grasping along the flight axis
                if grasp_points is not None:
                    # Get points in pixels (project back into image from 3D point)
                    p1 = np.asanyarray(self.cam.project(cam_intrinsics, grasp_points[0]))
                    p2 = np.asanyarray(self.cam.project(cam_intrinsics, grasp_points[1]))

                    img = cv2.circle(frame, (int(p1[0]), int(p1[1])), 3, (0,255,0))
                    img = cv2.circle(frame, (int(p2[0]), int(p2[1])), 3, (0,255,0))
                    # output_grasp.write(img)
                    delta_x = p1[0] - p2[0]
                    delta_y = np.abs(p1[1] - p2[1])

                    yaw = np.abs(np.arctan(delta_x/delta_y) * 180/np.pi - 90)
                
                # Use the computations from the point cloud
                if SEND_OUTPUT and not SIMPLE_LOC:
                   # Align to camera frame from Open3D frame axis assignment
                    tvec = [-centroid[0], centroid[1], -centroid[2], 1]
                   
                    # Same frame transformations as for simple localization
                    cam_2_drone_translation = [0.1267, -0.01, -0.09]
                    cam_2_drone_orientation = [0, -30, 0]

                    translation = [
                        self.quad_pose.x, self.quad_pose.y, self.quad_pose.z]
                    rotation = [
                        self.quad_pose.roll, -self.quad_pose.pitch, self.quad_pose.yaw]

                    # Align to motion capture frame axis assignment from camera frame 
                    tvec = [tvec[2], tvec[0], tvec[1], 1]
                    
                    # Transform into drone frame
                    tvec = transform_frame_EulerXYZ(cam_2_drone_orientation, cam_2_drone_translation, tvec, degrees=True)
                    # Transform into mocap frame
                    tvec = transform_frame_EulerXYZ(
                        rotation, translation, tvec, degrees=False)
                                    
                # Log output location and other metadata
                # self.logger.record_value([np.array(
                #         [tvec[0], tvec[1], tvec[2], self.elapsed_time, 0, class_name, self.quad_pose.x, self.quad_pose.y, self.quad_pose.z, self.quad_pose.roll, self.quad_pose.pitch, self.quad_pose.yaw]), ])
                print(f'logged {tvec}')
                self.success_frame_counter += 1


                # Different output filtering

                # Send mean of records
                length = len(self.logger.records[:,0].astype(float))
                if SEND_MEAN and length > 9:
                    x_mean = np.mean(self.logger.records[:, 0].astype(float))
                    y_mean = np.mean(self.logger.records[:, 1].astype(float))
                    z_mean = np.mean(self.logger.records[:, 2].astype(float))
                    msg.x = x_mean
                    msg.y = y_mean
                    msg.z = z_mean
                    

                # Send rolling average of last 10 records
                if SEND_ROLLING_AVG and length > 9:
                    num_records = 10 if length > 9 else length
                    x_avg = np.average(self.logger.records[-num_records:, 0].astype(float))
                    y_avg = np.average(self.logger.records[-num_records:, 1].astype(float))
                    z_avg = np.average(self.logger.records[-num_records:, 2].astype(float))
                    msg.x = x_avg
                    msg.y = y_avg
                    msg.z = z_avg

                # Send raw output
                if SEND_RAW or length < 10:
                    msg.x = tvec[0]
                    msg.y = tvec[1]
                    msg.z = tvec[2]
                    pass

                # Create rest of message and serialize
                print(yaw)
                msg.yaw = float(yaw)
                msg.label = class_name
                msg.confidence = 0.
                # serial_msg = msg.SerializeToString()
                
        # Time it took to analyse one frame 
        # elapsed_time = time.time() - starting_time

        # Show output frame
        if SHOW_WINDOW_VIS:
            self.output.write(vis_frame)
            cv2.imshow('output', vis_frame)
            cv2.waitKey(1)
        
        # Send output, if the target object wasn't detected, send default message
        if SEND_OUTPUT:
            if serial_msg is not None:
                self.publisher.publish(msg)
                # pass

            else:
                msg = Detection()
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.label = 'Nothing'
                msg.confidence = 0.0
                # serial_msg = msg.SerializeToString()
                self.publisher.publish(msg)
        

        # print(f'ELAPSED TIME (ms): {elapsed_time * 1000}')
        # times.append(elapsed_time)
        self.frame_counter += 1

    # def close(self):
    #     # Ctrl + C was pressed and we want to close gracefully
    #     # except KeyboardInterrupt as e:
    #     print('Closing handler called')
        
    #     # Send message to bridge process to stop running
    #     # msg = detection_msg_pb2.Detection()
    #     # msg.x = 0.0
    #     # msg.y = 0.0
    #     # msg.z = 0.0
    #     # msg.label = 'closing'
    #     # msg.confidence = 0.0
    #     # serial_msg = msg.SerializeToString()
    #     # socket.send(serial_msg)

    #     # Export logs of frame analysis times
    #     time_logger.records = np.asarray(times)
    #     time_logger.export_single_col_csv(f'logs/main_times_{utils.RECORD_COUNTER}.csv')
        
    #     # Close network connections and release video writers
    #     # Save logs
    #     receiver.close()
    #     self.output.release()
    #     self.output_depth.release()
    #     self.output_raw.release()
    #     self.output_grasp.release()
    #     self.cam.release()
    #     receiver.image_hub.close()
    #     cv2.destroyAllWindows()
    #     print(f'saving file to {utils.LOG_FILE}')
    #     logger.export_to_csv(utils.LOG_FILE)

# while True:
#     starting_time = time.time()
#     try:
#         if SEND_OUTPUT:
#             quad_pose_serial = socket.recv()
#             quad_pose = detection_msg_pb2.Detection()
#             quad_pose.ParseFromString(quad_pose_serial)

#         serial_msg = None

#         # Receive the frames from the ARM computer
#         frame, depth_frame = receiver.recv_frames()
  
#         cam_intrinsics = cam.intrinsics

#         # Write the RGB frame to the output without annotations        
#         output_raw.write(frame)

#         # Get detectron2 output
#         outputs = predictor(frame)
#         detected_class_idxs = outputs['instances'].pred_classes
#         pred_boxes = outputs['instances'].pred_boxes
        
#         # Show visualization if enabled
#         if SHOW_WINDOW_VIS:
#             out = v.draw_instance_predictions(frame, outputs['instances'].to('cpu'))
#             vis_frame = np.asarray(out.get_image())

#         # This holds the masks generated by detectron2
#         mask_array = outputs['instances'].pred_masks.to('cpu').numpy()
#         mask_array = np.moveaxis(mask_array, 0, -1)
#         num_instances = mask_array.shape[0]
#         mask_array_instance = []
        
#         # Loop over instances that have been detected
#         for i in range(num_instances):
#             class_idx = detected_class_idxs[i]
#             class_name = class_catalog[class_idx]
            
#             # Get bounding box if needed
#             bbox = pred_boxes[i].to('cpu')
#             [(xmin, ymin, xmax, ymax)] = bbox
#             # cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)
            

#             # Naive localization based on bounding box center
#             if SIMPLE_LOC:
#                 xmin = int(xmin)
#                 ymin = int(ymin)
#                 xmax = int(xmax)
#                 ymax = int(ymax)

#                 center_x = (xmax - xmin)//2 + xmin
#                 center_y = (ymax - ymin)//2 + ymin

#                 if center_y < cam.height and center_x < cam.width:
#                             depth = depth_frame[center_y,
#                                 center_x].astype(float)
#                             distance = depth * cam.depth_scale
#                 else:
#                     # No valid distance found
#                     distance = 0.0
#                     print('invalid coordinates')


#                 # Get translation vector relative to the camera frame
#                 tvec = cam.deproject(cam_intrinsics, center_x, center_y, distance)
#                 easy_center = tvec
                
#                 # Realsense y-axis points down by default
#                 tvec[1] = -tvec[1]
#                 yaw = 0

#             # Create the mask frame that can be applied on the RGB image
#             mask_array_instance.append(mask_array[:, :, i:(i+1)])
#             obj_mask = np.zeros_like(frame)
#             obj_mask = np.where(mask_array_instance[i] == True, 255, obj_mask)
#             # cv2.imwrite(f'pictures/mask_{class_name}.png',obj_mask)

#             # Target has been detected - apply frame transformation and/or send coordinates
#             if class_name == TARGET_OBJECT:
                
#                 # Simple localization - take naive estimate from bounding box
#                 if SEND_OUTPUT and SIMPLE_LOC:
#                     print('Object location (simple localization) -----')
                      
#                     # Holds the translation from the camera frame to the drone frame
#                     cam_2_drone_translation = [0.1267, -0.01, 0.0]
#                     # Holds the rotation from the camera frame to the drone frame
#                     cam_2_drone_orientation = [0, -30, 0]

#                     # Holds the translation from the drone frame to the global frame
#                     translation = [
#                         quad_pose.x, quad_pose.y, quad_pose.z]
#                     # Holds the rotation from the drone frame to the global frame
#                     rotation = [
#                         quad_pose.roll, -quad_pose.pitch, quad_pose.yaw]

#                     # Align axes accordingly to motion capture system
#                     # Turn into 4-vector for homogenous coordinates
#                     tvec = [tvec[2], tvec[0], tvec[1], 1]

                    
#                     # Transform into drone frame
#                     tvec = transform_frame_EulerXYZ(cam_2_drone_orientation, cam_2_drone_translation, tvec, degrees=True)

#                     # Transform into mocap frame
#                     tvec = transform_frame_EulerXYZ(
#                         rotation, translation, tvec, degrees=False)
                    
                    
#                 # Create Protobuf message to send object location
#                 msg = detection_msg_pb2.Detection()

                
#                 # Create point cloud of detected object

#                 # First, apply mask to RGB frame
#                 masked_frame = cv2.bitwise_and(frame, obj_mask)
#                 # cv2.imwrite('masked_frame.png', masked_frame)
                
#                 # Debugging
#                 if RECORD_PCD_DATA:
#                     grasp_color = GraspCandidate()
#                     grasp_masked = GraspCandidate()
                    
#                     grasp_masked.set_point_cloud_from_aligned_masked_frames(masked_frame, depth_frame, cam_intrinsics)
#                     grasp_color.set_point_cloud_from_aligned_frames(frame, depth_frame, cam_intrinsics)
                    
#                     grasp_masked.save_pcd(f'pcd/pcd_logs/{utils.RECORD_COUNTER}_{TARGET_OBJECT}_masked_{frame_counter}.pcd')
#                     grasp_color.save_pcd(f'pcd/pcd_logs/{utils.RECORD_COUNTER}_{TARGET_OBJECT}_full_{frame_counter}.pcd')
                    
#                     print(f'Recorded pcd for frame {frame_counter}, sleeping briefly')
#                     time.sleep(3)
                
                
#                 grasp_points = None
#                 try:
#                     # Create point cloud 
#                     grasp.set_point_cloud_from_aligned_masked_frames(masked_frame, depth_frame, cam_intrinsics)
#                     centroid = grasp.find_centroid()
#                     axis_ext, _, _ = grasp.find_largest_axis()
#                     # Get longest axis
#                     axis = axis_ext[0]
#                     # Rotate copy of point cloud around longest axis
#                     pcd = grasp.rotate_pcd_around_axis(grasp.pointcloud, centroid, math.pi, axis)
#                     grasp.pointcloud += pcd
#                     grasp.save_pcd(f'pcd/pointcloud_{TARGET_OBJECT}_{utils.RECORD_COUNTER}.pcd')
                    
#                     # Find grasping points
#                     grasp_points = grasp.find_grasping_points()
                    
#                 except Exception as e:
#                     # Sometimes, qhull fails and throws an error and we don't want the program to crash
#                     print('pcd data analysis went wrong')
#                     print(e)
                

#                 # Yaw computation needs to be adapted to current way of grasping along the flight axis
#                 if grasp_points is not None:
#                     # Get points in pixels (project back into image from 3D point)
#                     p1 = np.asanyarray(cam.project(cam_intrinsics, grasp_points[0]))
#                     p2 = np.asanyarray(cam.project(cam_intrinsics, grasp_points[1]))

#                     img = cv2.circle(frame, (int(p1[0]), int(p1[1])), 3, (0,255,0))
#                     img = cv2.circle(frame, (int(p2[0]), int(p2[1])), 3, (0,255,0))
#                     output_grasp.write(img)
#                     delta_x = p1[0] - p2[0]
#                     delta_y = np.abs(p1[1] - p2[1])

#                     yaw = np.abs(np.arctan(delta_x/delta_y) * 180/np.pi - 90)
                
#                 # Use the computations from the point cloud
#                 if SEND_OUTPUT and not SIMPLE_LOC:
#                    # Align to camera frame from Open3D frame axis assignment
#                     tvec = [-centroid[0], centroid[1], -centroid[2], 1]
                   
#                     # Same frame transformations as for simple localization
#                     cam_2_drone_translation = [0.1267, -0.01, -0.09]
#                     cam_2_drone_orientation = [0, -30, 0]

#                     translation = [
#                         quad_pose.x, quad_pose.y, quad_pose.z]
#                     rotation = [
#                         quad_pose.roll, -quad_pose.pitch, quad_pose.yaw]

#                     # Align to motion capture frame axis assignment from camera frame 
#                     tvec = [tvec[2], tvec[0], tvec[1], 1]
                    
#                     # Transform into drone frame
#                     tvec = transform_frame_EulerXYZ(cam_2_drone_orientation, cam_2_drone_translation, tvec, degrees=True)
#                     # Transform into mocap frame
#                     tvec = transform_frame_EulerXYZ(
#                         rotation, translation, tvec, degrees=False)
                                    
#                 # Log output location and other metadata
#                 logger.record_value([np.array(
#                         [tvec[0], tvec[1], tvec[2], elapsed_time, 0, class_name, quad_pose.x, quad_pose.y, quad_pose.z, quad_pose.roll, quad_pose.pitch, quad_pose.yaw]), ])
#                 print(f'logged {tvec}')
#                 success_frame_counter += 1


#                 # Different output filtering

#                 # Send mean of records
#                 length = len(logger.records[:,0].astype(float))
#                 if SEND_MEAN and length > 9:
#                     x_mean = np.mean(logger.records[:, 0].astype(float))
#                     y_mean = np.mean(logger.records[:, 1].astype(float))
#                     z_mean = np.mean(logger.records[:, 2].astype(float))
#                     msg.x = x_mean
#                     msg.y = y_mean
#                     msg.z = z_mean
                    

#                 # Send rolling average of last 10 records
#                 if SEND_ROLLING_AVG and length > 9:
#                     num_records = 10 if length > 9 else length
#                     x_avg = np.average(logger.records[-num_records:, 0].astype(float))
#                     y_avg = np.average(logger.records[-num_records:, 1].astype(float))
#                     z_avg = np.average(logger.records[-num_records:, 2].astype(float))
#                     msg.x = x_avg
#                     msg.y = y_avg
#                     msg.z = z_avg

#                 # Send raw output
#                 if SEND_RAW or length < 10:
#                     msg.x = tvec[0]
#                     msg.y = tvec[1]
#                     msg.z = tvec[2]
#                     pass

#                 # Create rest of message and serialize
#                 msg.yaw = yaw
#                 msg.label = class_name
#                 msg.confidence = 0
#                 serial_msg = msg.SerializeToString()
                
#         # Time it took to analyse one frame 
#         elapsed_time = time.time() - starting_time

#         # Show output frame
#         if SHOW_WINDOW_VIS:
#             output.write(vis_frame)
#             cv2.imshow('output', vis_frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
        
#         # Send output, if the target object wasn't detected, send default message
#         if SEND_OUTPUT:
#             if serial_msg is not None:
#                 socket.send(serial_msg)

#             else:
#                 msg = detection_msg_pb2.Detection()
#                 msg.x = 0.0
#                 msg.y = 0.0
#                 msg.z = 0.0
#                 msg.label = 'Nothing'
#                 msg.confidence = 0.0
#                 serial_msg = msg.SerializeToString()
#                 socket.send(serial_msg)
        

#         print(f'ELAPSED TIME (ms): {elapsed_time * 1000}')
#         times.append(elapsed_time)
#         frame_counter += 1

#     # Ctrl + C was pressed and we want to close gracefully
#     except KeyboardInterrupt as e:
#         print('Closing handler called')
        
#         # Send message to bridge process to stop running
#         msg = detection_msg_pb2.Detection()
#         msg.x = 0.0
#         msg.y = 0.0
#         msg.z = 0.0
#         msg.label = 'closing'
#         msg.confidence = 0.0
#         serial_msg = msg.SerializeToString()
#         socket.send(serial_msg)

#         # Export logs of frame analysis times
#         time_logger.records = np.asarray(times)
#         time_logger.export_single_col_csv(f'logs/main_times_{utils.RECORD_COUNTER}.csv')
        
#         # Close network connections and release video writers
#         # Save logs
#         receiver.close()
#         output.release()
#         output_depth.release()
#         output_raw.release()
#         output_grasp.release()
#         cam.release()
#         socket.close() # Setup for a custom logger
def main(args=None):
    rclpy.init(args=args)

    # The grasp object stores the point cloud of the object and implements grasp planning

    # rclpy.init()


    # VideoWriter outputs to store the analyzed frames at different points in time

    node = DetectionNode()
    rclpy.spin(node)
    node.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()