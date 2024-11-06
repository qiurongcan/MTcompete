
import rospy
import numpy as np
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from std_msgs.msg import Int32, Int32MultiArray
"""
创建一个专门用于控制飞机到达送货点后的节点 具有以下功能
1.释放货物
2.设置返回航线
TODO
1.设置无人机返航的路线
"""

class DroneNode(DemoPipeline):
    def __init__(self, node_name='qrcDroneNode', show=False):
        super().__init__(node_name, show)
    
        # 目前只保留了 1 3 4 5 6
        self.target_pos = {
            "pos1": Position(146, 186, -34), # 订单6 小车一
            "pos2": Position(430, 184, -10), # 订单1 小车六
            # "pos3": Position(528, 172, -20), # 订单2
            "pos4": Position(508, 514, -22), # 订单3 小车三
            "pos5": Position(564, 394, -16), # 订单4
            "pos6": Position(490, 390, -22), # 订单5
        }
        

        self.back_lock_sub = rospy.Subscriber("/back_lock", Int32MultiArray, self.back_lock_cb)

        self.back_lock_pub = rospy.Publisher("/back_lock_pub", Int32MultiArray, queue_size=10)
        self.msg = Int32MultiArray()

        # 【决赛】修改小车的返回路线
        # 返回至 2 3 4 5 6 小车
        self.fly_line ={
            'pos1': [Position(152, 182, -86), Position(110, 330, -86), Position(187, 431, -86), Position(187, 431, -21)],  #4 f
            'pos5': [Position(564, 402, -90), Position(194, 443, -90), Position(194, 438, -25)], #5 f
            'pos4': [Position(514, 519 ,-100), Position(181, 446, -100), Position(181, 440, -21)], #3 f
            'pos2': [Position(425, 182, -100), Position(244, 397, -100), Position(244, 397, -63), Position(207, 440, -63), Position(199, 440, -21)], # 6 f D
            # 'pos3': [Position(528, 172, -115), Position(387, 284, -115), Position(363, 308, -65), Position(199, 445, -65), Position(199, 440, -20)], #6 
            'pos6': [Position(490, 398, -96), Position(186, 438, -96), Position(186, 438, -25)], # 2 f
        }

        # 设置一个返回锁
        self.back_lock = [0, 0, 0, 0, 0, 0]

    def back_lock_cb(self, msg):
        self.back_lock = list(msg.data)

    def inspect_drone(self, drones):
        

        for drone in drones:
            drone_work_state = drone.drone_work_state
            drone_pos = drone.pos.position
            drone_sn = drone.sn
            for key, value in self.target_pos.items():
                if self.des_pos_reached(drone_pos, value, 2.0) and drone_work_state == 1:
                    if drone.bind_cargo_id != 0:
                        # 释放货物
                        rospy.loginfo(f"--{drone_sn}'s cargo_id is {drone.bind_cargo_id}--")
                        self.release_cargo(drone_sn, 1, WorkState.RELEASE_DRONE_RETURN)
                    
                    if self.back_lock[int(key[-1])-1] < 1:
                        self.fly_one_route(drone_sn, self.fly_line[key], 10, 2.0, WorkState.MOVE_CAR_GO_TO_PICKUP)
                        rospy.loginfo(f"---{drone_sn} is preparing for back---")
                        self.back_lock[int(key[-1])-1] += 1
                        
                        self.msg.data = self.back_lock
                        self.back_lock_pub.publish(self.msg)


    def main(self):
        rospy.sleep(4.0)
        self.sys_init()
        while not rospy.is_shutdown():
            self.inspect_drone(self.drone_physical_status)

 
if __name__ == "__main__":
    rospy.loginfo("---DroneNode is running---")
    drone = DroneNode()
    drone.main()


