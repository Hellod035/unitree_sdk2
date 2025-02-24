from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from _HeightMapData_ import HeightMapData_
import time

def LowStateHgHandler(msg: HeightMapData_):
    print("Recieve:", msg.height_map)


def height_map_data_init():
    return HeightMapData_([0.31] * 187)

ChannelFactoryInitialize(0, "eth0")

lowcmd_publisher = ChannelPublisher("test/height_map", HeightMapData_)
lowcmd_publisher.Init()

# lowstate_subscriber = ChannelSubscriber(
#     "test/height_map", HeightMapData_)
# lowstate_subscriber.Init(LowStateHgHandler, 10)


height_map_data = height_map_data_init()

while True:
    lowcmd_publisher.Write(height_map_data)
    time.sleep(1.0)

