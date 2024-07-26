from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
import time
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


class EncoderControll(Node):
    def __init__(self):
        super().__init__('encoder_controll')
        # 전역 변수로 present_voltage0, present_voltage1 선언
        self.present_voltage0 = 0.0
        self.present_voltage1 = 0.0
        
        # publisher 설정
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.voltageInput0 = VoltageInput()
        self.voltageInput1 = VoltageInput()

        self.voltageInput0.setChannel(0)
        self.voltageInput1.setChannel(1)

        self.voltageInput0.setOnVoltageChangeHandler(self.onVoltageChange0)
        self.voltageInput1.setOnVoltageChangeHandler(self.onVoltageChange1)
        
        self.voltageInput0.setOnErrorHandler(self.onError)
        self.voltageInput1.setOnErrorHandler(self.onError)
        
        self.voltageInput0.openWaitForAttachment(500)
        self.voltageInput1.openWaitForAttachment(500)

        time.sleep(3)
        self.init_position0 = ((float(self.present_voltage0) / (5 / 1024)) * 0.3515625)
        self.init_position1 = ((float(self.present_voltage1) / (5 / 1024)) * 0.3515625)
        self.init_voltage0 = self.present_voltage0
        self.init_voltage1 = self.present_voltage1

        print("complete ", self.init_position0, self.init_position1)


        # 타이머 설정 (0.1초마다 pub_encoder_val 호출)
        self.timer = self.create_timer(0.1, self.pub_encoder_val)
        
    def onVoltageChange0(self, voltageInput, voltage):
        self.present_voltage0 = voltage

    def onVoltageChange1(self, voltageInput, voltage):
        self.present_voltage1 = voltage

    def onError(self, code, description):
        print("Code [" + str(self.getChannel()) + "]: " + ErrorEventCode.getName(code))
        print("Description [" + str(self.getChannel()) + "]: " + str(description))
        print("----------")

    def pub_encoder_val(self):
        acctual_position0 = ((float(self.present_voltage0) / (5 / 1024)) * 0.3515625)
        acctual_position1 = ((float(self.present_voltage1) / (5 / 1024)) * 0.3515625)
        
        present_position0 = acctual_position0 - self.init_position0
        present_position1 = acctual_position1 - self.init_position1

        print(f"present 0 Position: {present_position0}")
        print(f"present 1 Position: {present_position1}")

        zero_area_linear = 6
        zero_area_angular = 3
        if -zero_area_angular < present_position0 < zero_area_angular:
            absolute_position0= 0
        else:
            if zero_area_angular < present_position0 :
                absolute_position0 = present_position0 - zero_area_angular
            elif present_position0 < -zero_area_angular:
                absolute_position0 = present_position0 + zero_area_angular

        if -zero_area_linear < present_position1 < zero_area_linear:
            absolute_position1 = 0
        else:
            if zero_area_linear < present_position1 :
                absolute_position1 = present_position1 - zero_area_linear
            elif present_position1 < -zero_area_linear:
                absolute_position1 = present_position1 + zero_area_linear

        pub_linear_vel = (absolute_position1/23)*0.5 
        pub_angler_vel = (absolute_position0/50)*1.5

        print(f"absolute 0 fix Position: {absolute_position1} + {pub_linear_vel} + {Vector3(x=pub_linear_vel, y=0.0, z=0.0)}")
        print(f"absolute 1 fix Position: {absolute_position0} + {pub_angler_vel} + {Vector3(x=0.0, y=0.0, z=pub_angler_vel)}")

        encoder_msg = Twist()
        encoder_msg.linear = Vector3(x=pub_linear_vel, y=0.0, z=0.0)
        encoder_msg.angular = Vector3(x=0.0, y=0.0, z=pub_angler_vel)
        self.publisher_.publish(encoder_msg)

def main():
    rclpy.init(args=None)
    node = EncoderControll()

    try:
        rclpy.spin(node)
    except (Exception, KeyboardInterrupt):
        pass
    finally:
        node.voltageInput0.close()
        node.voltageInput1.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
