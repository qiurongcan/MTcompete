

import rospy
import numpy as np
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from user_pkg.msg import UserCmdRequest
from std_msgs.msg import Int32, Int32MultiArray, String

"""
创建一个控制1号小车的节点:
1. 用于控制一号小车前往上货点
2. 一号小车载着飞机返回至起飞点后，起飞飞机，然后返回上货点
3. 如此往返执行
"""

class CarNode(DemoPipeline):
    def __init__(self, node_name='qrcCarNode', show=False):
        super().__init__(node_name, show)
        # TODO 修改为六辆车
        # 2号订单不送, 起飞前往目的地的路线
        # 0 为 6, 12, 18, 24 ...
        self.bills_drone_route = {
            "1": [Position(193, 431, -63), Position(434, 187, -63), Position(430, 184, -14)], # x: 430.0 y: 184.0 z: -10.0
            '3': [Position(193, 431, -72), Position(508, 514 ,-72), Position(508, 514 ,-26)], # x: 508.0 y: 514.0 z: -22.0
            '4': [Position(193, 431, -80), Position(564, 394, -80), Position(564, 394, -20)], # x: 564.0 y: 394.0 z: -16.0
            '5': [Position(193, 431, -75), Position(490, 390, -75), Position(490, 390, -26)], # x: 490.0 y: 390.0 z: -22.0
            '0': [Position(193, 431, -66), Position(130, 298, -72), Position(146, 186, -66), Position(146, 186, -38)], # x: 146.0 y: 186.0 z: -34.0
        }
        # ----------------------初始化移动的位置---------------------- # 
        # 一号小车先腾出一个位置
        self.one_car_init_route = [Position(183, 434, -16), Position(183, 431, -16), Position(187, 431, -16)]
        # 二号小车腾出一个位置
        self.two_car_init_route = [Position(190, 438, -16), Position(186, 438, -16)]
        # 三号小车腾出一个位置
        self.three_car_init_route = [Position(183, 446, -16), Position(181, 440, -16)]
        # 四号小车腾出一个位置
        self.four_car_init_route = [Position(197, 434, -16), Position(198, 431, -16), Position(193, 431, -16)]
        # 五号小车腾出一个位置
        self.five_car_init_route = [Position(190, 444, -16), Position(194, 438, -16)]
        # 六号小车腾出一个位置
        self.six_car_init_route = [Position(197, 446, 16), Position(199, 445, -16), Position(199, 440, -16)]
        # ---------------------------------------------------------- #

        # TODO【决赛】6辆小车都需要修改  1 和 4 之间或许可以用直线
        self.car_route = {
            # "SIM-MAGV-0001": [Position(187, 431, -16), Position(190, 425, -16)], # 1
            # "SIM-MAGV-0002": [Position(181, 431, -16), Position(185, 425, -16), Position(190, 425, -16)], # 2
            # "SIM-MAGV-0003": [Position(181, 431, -16), Position(185, 425, -16), Position(190, 425, -16)], # 3
            "SIM-MAGV-0004": [Position(193, 431, -16), Position(190, 425, -16)], # 4
            # "SIM-MAGV-0005": [Position(199, 431, -16), Position(195, 425, -16), Position(190, 425, -16)],  
            # "SIM-MAGV-0006": [Position(199, 431, -16), Position(195, 425, -16), Position(190, 425, -16)], 
        }
            

        # 【决赛】修改所有小车的初始位置 6号小车不动
        self.init_pos = {
            "SIM-MAGV-0001": Position(187, 431, -16), 
            "SIM-MAGV-0002": Position(181, 431, -16), 
            "SIM-MAGV-0003": Position(181, 431, -16), 
            "SIM-MAGV-0004": Position(193, 431, -16), # 只需要用到这辆车
            "SIM-MAGV-0005": Position(199, 431, -16), 
            "SIM-MAGV-0006": Position(199, 431, -16), 
        }
        
        self.unlock = 1
        # 上锁
        self.sub_lock = rospy.Subscriber("/lock", Int32, self.lock_cb)
        # 解锁
        self.sub_unlock = rospy.Subscriber("/unlock", Int32, self.unlock_cb)

        self.load_pos = Position(190, 425, -16)

        # 一号小车的移动标志
        self.move_flag = 0
        self.move_flag_sub = rospy.Subscriber("/move_flag", Int32, self.move_flag_cb)

    # 在此处更新小车的状态，用于破解循环锁
    def move_flag_cb(self, msg):
        self.move_flag = msg.data

    def lock_cb(self, msg):
        self.unlock = msg.data

    def unlock_cb(self, msg):
        self.unlock = msg.data


    def move_car_with_route(self, car_sn, route, time_est):
        """小车需要设置偏航角yaw"""
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point = route
        self.cmd_pub.publish(msg)
        rospy.loginfo(f'--{car_sn} is prepare for back --')
        rospy.sleep(time_est)
     

    
    # 控制一号小车
    def control_car(self):
        for car in self.car_physical_status:
            car_sn = car.sn
            carId = int(car_sn[-1]) - 1 # 转化为数字编号
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            car_drone_sn = car.drone_sn
            # TODO 重置move_flag
            if carId == 3 and car_work_state == 1: # 只运行四号
                if self.move_flag == 0 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 1): # 移动小车
                    # 小车前往上货点
                    if self.unlock:
                        self.move_car_with_route(car_sn, self.car_route[car_sn], 2.0)
                        self.move_flag = 1
                
                elif self.move_flag == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 1) and car_drone_sn != '':
                    for drone in self.drone_physical_status:
                        if car_drone_sn == drone.sn and drone.bind_cargo_id != 0:
                            # 路线根据订单号选择
                            # TODO 根据订单选择飞行路线，然后舍弃掉一条路线，只保留5个订单
                            bill_id = str(int(drone.bind_cargo_id % 6))
                            self.fly_one_route(car_drone_sn, self.bills_drone_route[bill_id], 10, 2.0, None)
                            rospy.loginfo(f"--------{car_drone_sn} Take off ---------")
                            self.move_flag = 0
                    
            else:
                continue
                
        


    def main(self):

        self.sys_init()
        self.state = WorkState.RUNNING_CAR
        # ----------------------小车初始布局----------------------
        # 1号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0001', route=self.one_car_init_route, time_est=1.0)
        # 2号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0002', route=self.two_car_init_route, time_est=2.0)
        # 3号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0003', route=self.three_car_init_route, time_est=2.0)
        # 4号小车可以先动
        self.move_car_with_route(car_sn='SIM-MAGV-0004', route=self.four_car_init_route, time_est=4.0)
        # 5号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0005', route=self.five_car_init_route, time_est=4.0)
        # 6号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0006', route=self.six_car_init_route, time_est=4.0)

        while not rospy.is_shutdown():

            self.control_car()

            # 设置优先顺序
            # car_msgs = self.sort_car_msg(self.car_physical_status)
            # self.inspect_car(car_msgs)
            



if __name__ == "__main__":
    rospy.loginfo("--CarNode is running--")
    car = CarNode()
    car.main()
