"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Spectacular AI ROS2 node that manages the OAK-D device through DepthAI Python API
"""

import spectacularAI
import depthai
import numpy as np

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu
from builtin_interfaces.msg import Time

import rclpy
import time
from rclpy.node import Node

import cv2

PUBLISHER_QUEUE_SIZE = 10

def toRosTime(timeInSeconds):
    t = Time()
    t.sec = int(timeInSeconds)
    t.nanosec = int((timeInSeconds % 1) * 1e9)
    return t


def toPoseMessage(cameraPose, ts):
    msg = PoseStamped()
    msg.header.stamp = ts
    msg.header.frame_id = "world"
    msg.pose.position.x = cameraPose.position.x
    msg.pose.position.y = cameraPose.position.y
    msg.pose.position.z = cameraPose.position.z
    msg.pose.orientation.x = cameraPose.orientation.x
    msg.pose.orientation.y = cameraPose.orientation.y
    msg.pose.orientation.z = cameraPose.orientation.z
    msg.pose.orientation.w = cameraPose.orientation.w
    return msg


def toTfMessage(cameraPose, ts, frame_id):
    msg = TFMessage()
    msg.transforms = []
    transform = TransformStamped()
    transform.header.stamp = ts
    transform.header.frame_id = "world"
    transform.child_frame_id = frame_id
    transform.transform.translation.x = cameraPose.position.x
    transform.transform.translation.y = cameraPose.position.y
    transform.transform.translation.z = cameraPose.position.z
    transform.transform.rotation.x = cameraPose.orientation.x
    transform.transform.rotation.y = cameraPose.orientation.y
    transform.transform.rotation.z = cameraPose.orientation.z
    transform.transform.rotation.w = cameraPose.orientation.w
    msg.transforms.append(transform)
    return msg


def toCameraInfoMessage(camera, frame, ts):
    intrinsic = camera.getIntrinsicMatrix()
    msg = CameraInfo()
    msg.header.stamp = ts
    msg.header.frame_id = "left_camera"
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.distortion_model = "none"
    msg.d = []
    msg.k = intrinsic.ravel().tolist()
    return msg

class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []
        # Add msg to array
        self.arrays[name].append({"msg": msg, "seq": msg.getSequenceNum()})

        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                if msg.getSequenceNum() == obj["seq"]:
                    synced[name] = obj["msg"]
                    break
        # If there are 5 (all) synced msgs, remove all old msgs
        # and return synced msgs
        if len(synced) == 2:  # color, left, right, depth, nn
            # Remove old msgs
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if obj["seq"] < msg.getSequenceNum():
                        arr.remove(obj)
                    else:
                        break
            return synced
        return False

class SpectacularAINode(Node):
    def __init__(self):
        super().__init__("spectacular_ai_node")
        self.declare_parameter('recordingFolder', rclpy.Parameter.Type.STRING)

        self.odometry_publisher = self.create_publisher(PoseStamped, "/slam/odometry", PUBLISHER_QUEUE_SIZE)
        # self.keyframe_publisher = self.create_publisher(PoseStamped, "/slam/keyframe", PUBLISHER_QUEUE_SIZE)
        # self.left_publisher = self.create_publisher(Image, "/slam/left", PUBLISHER_QUEUE_SIZE)
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", PUBLISHER_QUEUE_SIZE)
        # self.point_publisher = self.create_publisher(PointCloud2, "/slam/pointcloud", PUBLISHER_QUEUE_SIZE)
        # self.camera_info_publisher = self.create_publisher(CameraInfo, "/slam/camera_info", PUBLISHER_QUEUE_SIZE)

        self.cam1_publisher = self.create_publisher(Image, "/slam/cam1", PUBLISHER_QUEUE_SIZE)
        self.cam2_publisher = self.create_publisher(Image, "/slam/cam2", PUBLISHER_QUEUE_SIZE)
        self.depth_publisher = self.create_publisher(Image, "/slam/depth", PUBLISHER_QUEUE_SIZE)
        # self.features_publisher = self.create_publisher(Image, "/slam/features", PUBLISHER_QUEUE_SIZE)
        self.imu_publisher = self.create_publisher(Imu, "/slam/imu", PUBLISHER_QUEUE_SIZE)


        self.bridge = CvBridge()
        self.keyframes = {}
        self.latestOutputTimestamp = None
        self.sync = HostSync()

        self.pipeline = depthai.Pipeline()
        config = spectacularAI.depthai.Configuration()

        recordingFolder = str(self.get_parameter('recordingFolder').value)
        if recordingFolder:
            self.get_logger().info("Recording: " + recordingFolder)
            config.recordingFolder = recordingFolder
            config.recordingOnly = True

        config.internalParameters = {
            "ffmpegVideoCodec": "libx264 -crf 15 -preset ultrafast",
            "computeStereoPointCloud": "true",
            "computeDenseStereoDepthKeyFramesOnly": "true",
            "alreadyRectified": "true"
        }
        config.useSlam = False

        self.get_logger().info("Starting VIO") # Example of logging.
        # self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline, config, self.onMappingOutput)
        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline, config)
        self.featureBuffer = None

        self.vio_pipeline.hooks.imu = self.onImuData
        # self.vio_pipeline.hooks.monoPrimary = self.onImageFactor("Primary")
        # self.vio_pipeline.hooks.monoSecondary = self.onImageFactor("Secondary")
        # self.vio_pipeline.hooks.depth = self.onImageFactor("Depth")
        # self.vio_pipeline.hooks.trackedFeatures = self.onFeatures
        # In default mode the color is not used by the SDK, so this will never get called.
        # see mixed_reality.py example how to read the color data in an efficient manner.
        # self.vio_pipeline.hooks.color = self.onImageFactor("Color")

        # camRgb = self.pipeline.createColorCamera()
        # camRgb.setPreviewSize(416, 416)
        # camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
        # camRgb.setInterleaved(False)
        # camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)
        # xoutRgb = self.pipeline.createXLinkOut()
        # xoutRgb.setStreamName("rgb")
        # camRgb.preview.link(xoutRgb.input)

        # depthOut = self.pipeline.createXLinkOut()
        # depthOut.setStreamName("depth")
        # self.vio_pipeline.stereo.depth.link(depthOut.input)

        RGB_OUTPUT_WIDTH = 1024
        REF_ASPECT = 1920 / 1080.0
        FPS = 30
        w = RGB_OUTPUT_WIDTH
        h = int(round(w / REF_ASPECT))

        camRgb = self.pipeline.createColorCamera()
        camRgb.setPreviewSize(w, h)
        camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
        camRgb.setImageOrientation(depthai.CameraImageOrientation.VERTICAL_FLIP) # for OpenGL
        camRgb.setFps(FPS)
        camRgb.initialControl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
        camRgb.initialControl.setManualFocus(130) # seems to be about 1m
        out_source = camRgb.preview

        xout_camera = self.pipeline.createXLinkOut()
        xout_camera.setStreamName("rgb")
        out_source.link(xout_camera.input)

        xout_depth = self.pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        self.vio_pipeline.stereo.depth.link(xout_depth.input)

        self.device = depthai.Device(self.pipeline)

        self.vio_session = self.vio_pipeline.startSession(self.device)
        self.timer = self.create_timer(0, self.processOutput)
        # time.sleep(1)

    def processOutput(self):
        img_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        queues = [img_queue, depth_queue]
        for q in queues:
            new_msg = q.get()
            if new_msg is not None:
                msgs = self.sync.add_msg(q.getName(), new_msg)
                if msgs:
                    depth = msgs["depth"].getFrame()
                    color = msgs["rgb"].getCvFrame()

                    depth_vis = cv2.normalize(
                        depth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    depth_vis = cv2.equalizeHist(depth_vis)
                    depth_vis = cv2.applyColorMap(
                        depth_vis, cv2.COLORMAP_HOT)
                    cv2.imshow("depth", depth_vis)
                    cv2.imshow("color", color)
                    cv2.waitKey(1)

        while self.vio_session.hasOutput():
            self.onVioOutput(self.vio_session.getOutput())


    def onVioOutput(self, vioOutput):
        timestamp = toRosTime(vioOutput.getCameraPose(0).pose.time)
        self.latestOutputTimestamp = timestamp
        cameraPose = vioOutput.getCameraPose(0).pose
        self.odometry_publisher.publish(toPoseMessage(cameraPose, timestamp))
        self.tf_publisher.publish(toTfMessage(cameraPose, timestamp, "left_camera"))


    # def onMappingOutput(self, output):
    #     for frame_id in output.updatedKeyFrames:
    #         keyFrame = output.map.keyFrames.get(frame_id)
    #         if not keyFrame: continue # Deleted keyframe
    #         if not keyFrame.pointCloud: continue
    #         if not self.hasKeyframe(frame_id):
    #             self.newKeyFrame(frame_id, keyFrame)


    # def hasKeyframe(self, frame_id):
    #     return frame_id in self.keyframes


    # def newKeyFrame(self, frame_id, keyframe):
    #     if not self.latestOutputTimestamp: return
    #     timestamp = toRosTime(keyframe.frameSet.primaryFrame.cameraPose.pose.time)
    #     self.keyframes[frame_id] = True
    #     msg = toPoseMessage(keyframe.frameSet.primaryFrame.cameraPose.pose, timestamp)
    #     msg.header.stamp = timestamp
    #     self.keyframe_publisher.publish(msg)

    #     left_frame_bitmap = keyframe.frameSet.primaryFrame.image.toArray()
    #     left_msg = self.bridge.cv2_to_imgmsg(left_frame_bitmap, encoding="mono8")
    #     left_msg.header.stamp = timestamp
    #     left_msg.header.frame_id = "left_camera"
    #     self.left_publisher.publish(left_msg)

    #     camera = keyframe.frameSet.primaryFrame.cameraPose.camera
    #     info_msg = toCameraInfoMessage(camera, left_frame_bitmap, timestamp)
    #     self.camera_info_publisher.publish(info_msg)

    #     self.publishPointCloud(keyframe, timestamp)


    # NOTE This seems a bit slow.
    # def publishPointCloud(self, keyframe, timestamp):
    #     camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix()
    #     positions = keyframe.pointCloud.getPositionData()
    #     pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
    #     p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
    #     pc[:, :3] = (camToWorld @ p_C[:, :, None])[:, :3, 0]

    #     msg = PointCloud2()
    #     msg.header.stamp = timestamp
    #     msg.header.frame_id = "world"
    #     if keyframe.pointCloud.hasColors():
    #         pc[:, 3:] = keyframe.pointCloud.getRGB24Data() * (1. / 255.)
    #     msg.point_step = 4 * 6
    #     msg.height = 1
    #     msg.width = pc.shape[0]
    #     msg.row_step = msg.point_step * pc.shape[0]
    #     msg.data = pc.tobytes()
    #     msg.is_bigendian = False
    #     msg.is_dense = False
    #     ros_dtype = PointField.FLOAT32
    #     itemsize = np.dtype(np.float32).itemsize
    #     msg.fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzrgb')]
    #     self.point_publisher.publish(msg)
    
    # def onImageFactor(self, name):
    #     def onImage(img):
    #         # if img.getWidth() <= 0 or img.getHeight() <= 0:
    #         #     # When SLAM is enabled, monocular frames are only used at 1/6th of normal frame rate,
    #         #     # rest of the frames are [0,0] in size and must be filtered
    #         #     return
    #         # if type(self.featureBuffer) is not np.ndarray: self.featureBuffer = np.zeros((img.getHeight(), img.getWidth(), 1), dtype = "uint8")
    #         # cv2.imshow(name, img.getCvFrame())
    #         # if cv2.waitKey(1) == ord("q"):
    #         #     exit(0)
    #         # publish using camera publisher
    #         if name == "Primary":
    #             self.publisher = self.cam1_publisher
    #         elif name == "Secondary":
    #             self.publisher = self.cam2_publisher
    #         elif name == "Depth":
    #             self.publisher = self.depth_publisher
    #         # elif name == "Color":
    #         #     self.publisher = self.features_publisher
    #         self.publisher.publish(self.bridge.cv2_to_imgmsg(img.getCvFrame(), encoding="passthrough"))
    #     return onImage

    def onImuData(self, imuData):
        for imuPacket in imuData.packets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope
            acceleroTs = acceleroValues.getTimestampDevice().total_seconds() * 1000
            gyroTs = gyroValues.getTimestampDevice().total_seconds() * 1000
            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.angular_velocity.x = gyroValues.x
            imu_msg.angular_velocity.y = gyroValues.y
            imu_msg.angular_velocity.z = gyroValues.z
            imu_msg.linear_acceleration.x = acceleroValues.x
            imu_msg.linear_acceleration.y = acceleroValues.y
            imu_msg.linear_acceleration.z = acceleroValues.z
            self.imu_publisher.publish(imu_msg)
            # print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
            # print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
            # print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
            # print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")

    # def onFeatures(self, features):
    #     if type(self.featureBuffer) is not np.ndarray: return
    #     self.featureBuffer[:] = 0
    #     # for feature in features.trackedFeatures:
    #     #     cv2.circle(self.featureBuffer, (int(feature.position.x), int(feature.position.y)), 2, 255, -1, cv2.LINE_AA, 0)
    #     # cv2.imshow("Features", self.featureBuffer)
    #     # if cv2.waitKey(1) == ord("q"):
    #     #     exit(0)

    #     # publish features image



def main(args=None):
    rclpy.init(args=args)
    sai_node = SpectacularAINode()
    rclpy.spin(sai_node)
    sai_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
