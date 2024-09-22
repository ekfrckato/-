import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import numpy as np
from scipy.interpolate import interp1d
import cv2
from scipy.signal import lfilter

class DepthArraySubscriber(Node):
    def __init__(self):
        super().__init__('depth_array_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray,'/depth_array_msg',self.opencv_graph,10)
        self.subscription  # prevent unused variable warning
        #publisher setting
        self.publisher_diff_array = self.create_publisher(Float32MultiArray, '/diff_array_msg', 10)
        self.publisher_depth_img = self.create_publisher(Float32MultiArray, '/depth_imp_img', 10)
        self.publisher_stairs_check = self.create_publisher(String , '/stairs_check', 10)
        self.get_data = []
        
    def capture_data(self, gradients):
        self.get_data.add(gradients) 
        
    
    def opencv_graph(self, msg):
        depth_array = msg.data
        
        height = 240
        width = 320
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        if len(depth_array) == 0:
            cv2.imshow('Depth Array', img)
            cv2.waitKey(1)
            return
        
        x = np.arange(len(depth_array))
        y = np.array(depth_array)

        # 0값에 가까운 잡음 처리
        noise_value_num = [i for i in range(len(y)) if np.isclose(y[i], 0.0, atol=1e-6)] 
        for idx in noise_value_num:
            if idx == 0:
                y[idx] = y[idx + 1] if (idx + 1) < len(y) else y[idx]
            elif idx == len(y) - 1:
                y[idx] = y[idx - 1]
            else:
                y[idx] = y[idx + 1] if not np.isclose(y[idx + 1], 0.0, atol=1e-6) else y[idx - 1]

        # 스케일 조정을 위해 y 값 조정
        diff = np.max(y) - np.min(y)
        scale_factor = height / diff if diff > 0 else 1
        y_scaled = (y - np.min(y)) * scale_factor
        y_scaled = height - y_scaled  # Y축 반전 적용

        # 기울기 계산
        gradients = np.diff(y_scaled)
        print(gradients)

        # 퍼블리시 관련 부분은 그대로 유지
        gradients2list = gradients.tolist()
        gradient_msg = Float32MultiArray()
        gradient_msg.data = gradients2list
        self.publisher_diff_array.publish(gradient_msg)

        # 계단 여부 체크 후 퍼블리시
        wall_diff = 3
        wall_height = 3
        floor_diff = 1
        floor_width = 3
        index_wall_N = 0
        index_floor_N = 0
        stair_being_there = False

        for i in range(len(gradients)):
            # 15픽셀 이상이고 -3.6 이하의 기울기 조건
            if gradients[i] > wall_diff:
                index_wall_N += 1
            
            # 벽의 높이가 15픽셀 이상인 경우
            if index_wall_N >= wall_height:
                # 벽 이후 10픽셀 이상이고 1 이상의 기울기가 연속적으로 나오는지 확인
                for j in range(i + 1, len(gradients)):
                    if gradients[j] < floor_diff and (j - i) > floor_width:
                        index_floor_N += 1
                        break
            
            # 벽이 15픽셀 이상이고, 10픽셀 이상 1 이상의 기울기가 연속적으로 나오는 경우
            if index_wall_N >= wall_height and index_floor_N >= 1:
                stair_being_there = True
                break

        if stair_being_there:
            print('계단이 있음')
            stairs_check = String()
            stairs_check.data = 'over'
            self.publisher_stairs_check.publish(stairs_check)
        # xnew와 y_scaled 그래프 그리기 부분
        if np.size(y) == 0:
            xnew = np.linspace(0, 1, 1)
        else:
            xnew = np.linspace(0, np.size(y) - 1, num=np.size(y))

        prev_point = None
        max_xnew = np.max(xnew)
        if np.isnan(max_xnew) or max_xnew == 0:
            print("Invalid max value for xnew")
            return

        # 퍼블리시 제거, 그래프만 그리도록 변경
        for i, (x_val, y_val) in enumerate(zip(xnew, y_scaled)):
            x_curr = int(width - ((x_val / max_xnew) * width))
            y_curr = int(y_val)
            curr_point = (x_curr, y_curr)
            
            if prev_point is not None:
                cv2.line(img, prev_point, curr_point, (0, 255, 0), 2)
            
            prev_point = curr_point
        
        cv2.imshow('Depth Array', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    depth_array_subscriber = DepthArraySubscriber()

    try:
        rclpy.spin(depth_array_subscriber)
    except KeyboardInterrupt:
        print('Depth array subscriber stopped cleanly')
    except BaseException as e:
        print(f'Exception in depth array subscriber: {e}')
        raise
    finally:
        depth_array_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
