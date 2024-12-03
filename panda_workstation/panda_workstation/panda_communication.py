import rclpy
from rclpy.node import Node
from panda_info.msg import PandaWsMsg  # Import the custom message
from colorama import Fore, Back, Style

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('panda_communication')
        self.publisher_ = self.create_publisher(PandaWsMsg, 'PCBTransferInfo', 10)
        timer_period = 2.0  # Publish every 2 seconds
        self.timer = self.create_timer(timer_period, self.publish_custom_message)

    def publish_custom_message(self):
        # Create a message object
        msg = PandaWsMsg()
        # Fill in header
        msg.id = "PCB_015"
        msg.departured = "AGV Sent"
        # Fill in body
        msg.width = 0.125
        msg.height = 0.1
        msg.defected = True
        msg.material_info = "Accroding to data sheet"
        msg.heatsink_number = 60


        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(Fore.GREEN + f'Publishing Info To Disassembly Station: \n \tid={msg.id},\n \t departured={msg.departured}, \n \t width={msg.width},\n \t height={msg.height}, \n \t defected={msg.defected}, \n \t material_info={msg.material_info},\n \t heatsink_number={msg.heatsink_number},\n \t defect loc x={msg.defect_loc_x},\n \t defect loc y={msg.defect_loc_y}')

def main(args=None):
    rclpy.init(args=args)
    custom_publisher = CustomPublisher()
    custom_publisher.publish_custom_message()
    try:
        rclpy.spin(custom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
    # Shutdown the node and clean up resources
        custom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

