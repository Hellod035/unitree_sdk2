import rospy
import tf2_ros
from grid_map_msgs.msg import GridMap
from scipy.ndimage import rotate
from tf.transformations import euler_from_quaternion
import signal
import sys
import copy
from threading import Lock
import numpy as np
import time
import os
from scipy import stats
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
import time
from unitree_sdk2py.utils.thread import RecurrentThread

def signal_handler(sig, frame):
    print('shutdown')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


class Map_Subscriber:
    def __init__(self):
        
        ChannelFactoryInitialize(0, "eth0")
        self.lowcmd_publisher = ChannelPublisher("height_map/height_map_data", String_)
        self.lowcmd_publisher.Init()

        rospy.init_node('getmap_node', anonymous=True)
        rospy.Subscriber('/elevation_mapping/elevation_map_filter', GridMap, self.elevation_map_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pelvis_height_init = 0.31
        
    # map pelvis
    def get_trans_and_rot(self, target_frame="odom", source_frame="base_footprint"):
        try:
            # 获取变换
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            return translation, rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform from %s to %s not found: %s", source_frame, target_frame, str(e))
            return None, None

    def elevation_map_callback(self, msg):
        # print("into callback")
        row_size = msg.data[0].layout.dim[1].size
        col_size = msg.data[0].layout.dim[0].size
        layer_data = np.array(msg.data[0].data).reshape(col_size, row_size)
        layer_data = layer_data.T

        rot = None
        trans = None
        trans, rot = self.get_trans_and_rot()

        _, _, yaw = euler_from_quaternion(rot)
        angle_deg = yaw * 180.0 / 3.141592653589793
        robot_frame_map = rotate(layer_data, angle=-angle_deg, reshape=False, order=1)

        # 形状变化
        crop_height = 17
        crop_width = 11
        start_x = (robot_frame_map.shape[0] - crop_height) // 2
        start_y = (robot_frame_map.shape[1] - crop_width) // 2
        robot_frame_map = robot_frame_map[start_x:start_x + crop_height, start_y:start_y + crop_width]
        robot_frame_map = np.flipud(robot_frame_map)
        robot_frame_map = np.fliplr(robot_frame_map)
        robot_frame_map = robot_frame_map.T
        
        # 数值变化
        robot_frame_map = self.pelvis_height_init - robot_frame_map + trans[2]
        robot_frame_map[robot_frame_map < -0.3] = self.pelvis_height_init
        robot_frame_map_str = np.array2string(robot_frame_map.reshape(-1), separator=',', precision=6)
        height_map_data = String_(robot_frame_map_str)
        # timestamp = int(time.time())  # 使用时间戳命名文件
        # filename = os.path.join("height_maps", f"height_map_{timestamp}.csv")
        # np.savetxt(filename, robot_frame_map, delimiter=',', fmt='%.4f')
        # print(f"Height map saved to {filename}")
        self.lowcmd_publisher.Write(height_map_data)
        return 


class Controller_example:
    def __init__(self):
        self.node = Map_Subscriber()



if __name__ == "__main__":
    controller = Controller_example()
    cnt = 0
    while True:
        time.sleep(10.0)
    