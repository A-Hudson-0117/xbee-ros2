import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from xbee_msgs.msg import XBeeSend

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.models.status import TransmitStatus
from digi.xbee.exception import *


# Use broadcast address (0x000000000000FFFF)
BROADCAST_ADDR = XBee64BitAddress.BROADCAST_ADDRESS



class XBeeROSNode(Node):
    def __init__(self):
        super().__init__('xbee_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        # XBee Setup
        self.device = XBeeDevice(port, baud)
        self.device.open()
        
        # set remote broadcast
        self.remote_broadcast = RemoteXBeeDevice(self.device, BROADCAST_ADDR)
        
        # set data recieve callback
        self.device.add_data_received_callback(self.on_receive)
        
        # ROS interfaces # ROS interfaces
        self.recv_pub = self.create_publisher(String, 'xbee/recv', 10)
        self.send_sub = self.create_subscription(XBeeSend, 'xbee/send', self.on_send_request, 10)
        self.broadcast_sub = self.create_subscription(String, 'xbee/broadcast', self.on_broadcast_request, 10)
        
        self.get_logger().info(f"XBee opened on {port} @ {baud} baud.")
        
    
        
    def destroy_node(self):
        self.get_logger().info("Shutting down XBee node...")
        if self.device.is_open():
            self.device.close()
        super().destroy_node()
    
    # ------------------------------
    # ROS receive callback
    # ------------------------------
    def on_receive(self, xbee_message):
        try:
            incoming_addr = str(xbee_message.remote_device.get_64bit_addr())
            data = xbee_message.data.decode()
            self.get_logger().info(f"[RX] {incoming_addr} : {data}")
            msg = String()
            msg.data = data
            self.recv_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error decoding message: {e}")
    
    # ------------------------------
    # ROS broadcast callback
    # ------------------------------
    def on_broadcast_request(self, ros_msg):
        try:
            self.device.send_data_async(self.remote_broadcast, ros_msg.data.encode())
            self.get_logger().info(f"[TX] broadcast : {ros_msg.data}")
        except Exception as e:
            self.get_logger().error(f"Broadcast send failed: {e}")
    
    # ------------------------------
    # ROS transmit callback
    # ------------------------------
    def on_send_request(self, ros_msg):
        encoded = ros_msg.data.encode()
        destination = ros_msg.destination
        fallback = False

        try:
            if destination:
                remote = RemoteXBeeDevice(self.device, XBee64BitAddress.from_hex_string(destination))
                self.device.send_data_async(remote, encoded)
                self.get_logger().info(f"[TX] {destination} : {ros_msg.data}")
            else:
                fallback = True
        except Exception as e:
            fallback = True

        if fallback:
            self.get_logger().warn(f"[Fallback] Broadcasting : '{ros_msg.data}'")
            self.on_broadcast_request(ros_msg)





def main(args=None):
    rclpy.init(args=args)
    node = XBeeROSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == "__main__": main()