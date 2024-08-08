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
        self.zero_area_linear = 6
        self.zero_area_angular = 9
        
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
        self.timer = self.create_timer(0.01, self.pub_encoder_val)
        
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
        
        acctual_angular = acctual_position0 - self.init_position0
        acctual_linear = acctual_position1 - self.init_position1

        # print(f"present 0 Position: {acctual_angular}")
        # print(f"present 1 Position: {acctual_linear}")

        #define safe controll area
        if -self.zero_area_angular < acctual_angular < self.zero_area_angular:
            angular= 0
        else:
            if self.zero_area_angular < acctual_angular :
                angular = acctual_angular - self.zero_area_angular
            elif acctual_angular < -self.zero_area_angular:
                angular = acctual_angular + self.zero_area_angular

        if -self.zero_area_linear < acctual_linear < self.zero_area_linear:
            linear = 0
        else:
            if self.zero_area_linear < acctual_linear :
                linear = acctual_linear - self.zero_area_linear
            elif acctual_linear < -self.zero_area_linear:
                linear = acctual_linear + self.zero_area_linear

        linear_max = 0.5
        angular_max = 1.5
        linear = (linear/23)*0.5 
        angular = (angular/50)*1.5


        if linear == 0:
            pub_angler_vel = angular
        else :
            pub_angler_vel = angular * linear / linear_max
        pub_linear_vel = linear

        # if linear == 0 and angular > 0.25 :


        print(f"linear: {linear} + cmd : {Vector3(x=pub_linear_vel, y=0.0, z=0.0)}")
        print(f"angular: {angular} + cmd : {Vector3(x=0.0, y=0.0, z=pub_angler_vel)}")

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

if __name__ == '__main__':
    main()
