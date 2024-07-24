from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
import time
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node


class EncoderControll(Node):
    def __init__(self):
        super().__init__('encoder_controll')
        # 전역 변수로 present_voltage0, present_voltage1 선언
        self.present_voltage0 = 0.0
        self.present_voltage1 = 0.0
        
        #publisher setting
        self.publisher_ = self.create_publisher(Float32MultiArray, '/encoder_val', 10)

        self.voltageInput0 = VoltageInput()
        self.voltageInput1 = VoltageInput()

        self.voltageInput0.setChannel(0)
        self.voltageInput1.setChannel(1)

        self.voltageInput0.setOnVoltageChangeHandler(self.onVoltageChange)
        self.voltageInput0.setOnErrorHandler(self.onError)
        self.voltageInput0.openWaitForAttachment(500)
        
        self.voltageInput1.setOnVoltageChangeHandler(self.onVoltageChange)
        self.voltageInput1.setOnErrorHandler(self.onError)
        self.voltageInput1.openWaitForAttachment(500)

        time.sleep(1)
        self.init_position0 = ((float(self.present_voltage0) / (5 / 1024)) * 0.3515625)
        self.init_position1 = ((float(self.present_voltage1) / (5 / 1024)) * 0.3515625)


        # 타이머 설정 (0.1초마다 pub_encoder_val 호출)
        self.timer = self.create_timer(0.1, self.pub_encoder_val)
        
    def onVoltageChange(self, voltageInput, voltage):
        if voltageInput.getChannel() == 0:
            self.present_voltage0 = voltage
        elif voltageInput.getChannel() == 1:
            self.present_voltage1 = voltage

    def onError(self, code, description):
        print("Code [" + str(self.getChannel()) + "]: " + ErrorEventCode.getName(code))
        print("Description [" + str(self.getChannel()) + "]: " + str(description))
        print("----------")

    def pub_encoder_val(self):
        present_position0 = ((float(self.present_voltage0) / (5 / 1024)) * 0.3515625)
        present_position1 = ((float(self.present_voltage1) / (5 / 1024)) * 0.3515625)
        encoder_msg = Float32MultiArray()
        encoder_msg.data = [present_position0, present_position1]
        
        print(self.init_position0, self.init_position1)

        print(f"Channel 0 Position: {present_position0} + {self.present_voltage0}")
        print(f"Channel 1 Position: {present_position1} + {self.present_voltage1}")
        
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
