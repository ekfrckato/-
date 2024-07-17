import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
import sys, tty, termios
from dynamixel_sdk import * # Uses Dynamixel SDK library

class StairsCheckSubscriber(Node):
    def __init__(self):
        super().__init__('stairs_check_subsciber')
        self.sub_stair = self.create_subscription(String, ' /stairs_check', self.stair_rocomotion_control, 10)
        self.sub_stair

        self.MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        self.BAUDRATE                    = 57600
        self.PROTOCOL_VERSION            = 2.0
        
        self.DXL_ID                      = 1
        self.DEVICENAME                  = '/dev/ttyACM0'
        self.TORQUE_ENABLE               = 1     # Value for enabling the torque
        self.TORQUE_DISABLE              = 0     # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.COMM_SUCCESS = 0
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()
            
        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()

        def getch(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        self.write_status(dxl_comm_result, dxl_error)
        if self.write_status(dxl_comm_result, dxl_error) == True : print("Dynamixel has been successfully connected")

        while 1:
            print("Press any key to continue! (or press ESC to quit!)")
            if self.getch() == chr(0x1b):
                break

        def write_status(self, dxl_comm_result, dxl_error):
            if dxl_comm_result != self.COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def stair_rocomotion_control(self, msg):
        if (msg.data == 'there is stairs') : stair_check_num = 1
        else : stair_check_num = 0

        dxl_goal_position = [1000,3000 ]         # Goal position

        if stair_check_num == 1:
            index = 0
            drive_mode = 10
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, drive_mode, 0)#normal mode 
            self.write_status(dxl_comm_result, dxl_error)
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
            self.write_status(dxl_comm_result, dxl_error)
            xl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, dxl_goal_position[index])
            self.write_status(dxl_comm_result, dxl_error)

            while abs(dxl_goal_position[index] - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
                # Read present position
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
                self.write_status(dxl_comm_result, dxl_error)
                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.DXL_ID, dxl_goal_position[index], dxl_present_position))
                # Small delay to prevent spamming the read request
                time.sleep(0.1)

            print("Reached goal position")
            index += 1
    
def main(args=None):
    rclpy.init(args=args)

    stairs_check = StairsCheckSubscriber()

    try:
        rclpy.spin(stairs_check)
    except KeyboardInterrupt:
        print('Depth array subscriber stopped cleanly')
    except BaseException as e:
        print(f'Exception in depth array subscriber: {e}')
        raise
    finally:
        StairsCheckSubscriber.destroy_node()
        rclpy.shutdown()

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = StairsCheckSubscriber.packetHandler.write1ByteTxRx(StairsCheckSubscriber.portHandler, StairsCheckSubscriber.DXL_ID, StairsCheckSubscriber.ADDR_TORQUE_ENABLE, StairsCheckSubscriber.TORQUE_DISABLE)
    StairsCheckSubscriber.write_status( dxl_comm_result, dxl_error)

    portHandler = PortHandler(StairsCheckSubscriber.DEVICENAME)
    packetHandler = PacketHandler(StairsCheckSubscriber.PROTOCOL_VERSION)
    StairsCheckSubscriber.COMM_SUCCESS = 0
    # Close port
    StairsCheckSubscriber.portHandler.closePort()


if __name__ == '__main__':
    main()



        
        


