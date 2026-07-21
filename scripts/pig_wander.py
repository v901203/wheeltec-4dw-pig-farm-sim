#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class PigWanderNode(Node):
    def __init__(self, num_pigs=16):
        super().__init__('pig_wander_node')
        self.num_pigs = num_pigs
        self.publishers_ = []
        
        # 為 16 隻豬建立專屬的 cmd_vel 發布者
        for i in range(self.num_pigs):
            topic_name = f'/model/farm_pig_{i}/cmd_vel'
            pub = self.create_publisher(Twist, topic_name, 10)
            self.publishers_.append(pub)
            
        # 縮短更新頻率到 1.5 秒，讓轉頭動作更流暢
        self.timer = self.create_timer(1.5, self.timer_callback)
        self.get_logger().info(f"🐖 豬隻「純旋轉」大腦已啟動！(共控制 {num_pigs} 隻)")

    def timer_callback(self):
        for pub in self.publishers_:
            msg = Twist()
            # 絕對禁止往前走
            msg.linear.x = 0.0 
            
            # 60% 機率活動，40% 機率原地發呆
            if random.random() < 0.6:
                # 使用高斯分佈產生角速度 (平均值 0.0，標準差 0.5)
                # 這樣轉向會比較自然，不會每次都猛轉
                msg.angular.z = random.gauss(0.0, 0.5) 
            else:
                msg.angular.z = 0.0
            
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # 這裡確保參數設定為 16 隻
    node = PigWanderNode(num_pigs=16) 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()