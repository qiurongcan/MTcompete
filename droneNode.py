
import rospy
import numpy as np
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position

"""
创建一个专门用于控制飞机到达送货点后的节点 具有以下功能
1.释放货物
2.设置返回航线
"""

class DroneNode(DemoPipeline):
    def __init__(self, node_name='qrcDroneNode', show=False):
        super().__init__(node_name, show)
    
        self.target_pos = {
            "pos1": Position(146, 186, -34), # 订单6
            "pos2": Position(430, 184, -10), # 订单1
            "pos3": Position(528, 172, -20), # 订单2
            "pos4": Position(508, 514, -22), # 订单3
            "pos5": Position(564, 394, -16), # 订单4
            "pos6": Position(490, 390, -22), # 订单5
        }

        # 【决赛】修改小车的返回路线
        # 存储六个目的地的航线
        self.fly_line ={
            'pos1': [Position(148, 184, -66), Position(130, 298, -66), Position(186, 438, -85),Position(186, 438, -20)], #5
            'pos5': [Position(564, 394, -85), Position(187, 431, -80), Position(187, 431, -20)],# 2
            'pos4': [Position(508, 514 ,-75), Position(193, 431, -66), Position(193, 431, -20)], # 1
            'pos2': [Position(430, 184, -70), Position(199, 431, -70), Position(199, 431, -20)], # 4
            'pos3': [Position(528, 172, -115), Position(196, 446, -115), Position(196, 446, -20)], #6 不动
            'pos6': [Position(490, 390, -80), Position(181, 440, -75), Position(181, 440, -20)], # 3
        }


    def inspect_drone(self, drones):
        for drone in drones:
            drone_work_state = drone.drone_work_state
            drone_pos = drone.pos.position
            drone_sn = drone.sn
            for key, value in self.target_pos.items():
                if self.des_pos_reached(drone_pos, value, 2.0) and drone_work_state == 1:
                    # if drone.bind_cargo_id != 0:
                        # 释放货物
                    rospy.loginfo(f"--{drone_sn}'s cargo_id is {drone.bind_cargo_id}--")
                    self.release_cargo(drone_sn, 1, WorkState.RELEASE_DRONE_RETURN)
                    self.fly_one_route(drone_sn, self.fly_line[key], 10, 2.0, WorkState.MOVE_CAR_GO_TO_PICKUP)
                    rospy.loginfo(f"---{drone_sn} is preparing for back---")


    def main(self):
        rospy.sleep(4.0)
        self.sys_init()
        while not rospy.is_shutdown():
            self.inspect_drone(self.drone_physical_status)
            rospy.sleep(0.04)

 
if __name__ == "__main__":
    rospy.loginfo("---DroneNode is running---")
    drone = DroneNode()
    drone.main()


