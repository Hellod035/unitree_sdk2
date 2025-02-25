from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
import time

def LowStateHgHandler(msg: String_):
    print("Recieve:", msg.data)


def height_map_data_init():
    return String_(str([0.31]*187))

ChannelFactoryInitialize(0, "eth0")

lowcmd_publisher = ChannelPublisher("height_map/height_map_data", String_)
lowcmd_publisher.Init()

lowstate_subscriber = ChannelSubscriber(
    "height_map/height_map_data", String_)
lowstate_subscriber.Init(LowStateHgHandler, 10)


height_map_data = height_map_data_init()

while True:
    lowcmd_publisher.Write(height_map_data)
    time.sleep(1.0)

