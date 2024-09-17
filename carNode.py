

import rospy
import numpy as np
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from user_pkg.msg import UserCmdRequest

"""
创建一个专门控制小车的节点 具有以下功能：
1.初始状态靠边
2.初始开始去接小车
3.航线上有飞机则等待飞机
4.航线上没有飞机且自身没有飞机，则前往上货点装飞机和上货
5.航线上没有飞机但是自身有飞机，则前往上货点上货
6.航线上没有飞机，但自身有飞机，且飞机上有货物，则放飞飞机，然后等待飞机
"""


class CarNode(DemoPipeline):
    def __init__(self, node_name='qrcCarNode', show=False):
        super().__init__(node_name, show)

        """
        car     cargo
        1(184, 434, -16)  -->  1 (146, 186, -34)    z = -66
        2(184, 440, -16)  -->  5 (564, 394, -16)    z = -80  直线
        3(184, 446, -16)  -->  4 (508, 514, -22)    z = -72  直线
        4(196, 434, -16)  -->  2 (430, 184, -10)    z = -100 直线
        5(196, 440, -16)  -->  3 (528, 172, -20)    z = -115 直线
        6(196, 446, -16)  -->  6 (490, 390, -22)    z = -90 直线
        """

        self.car_route = {
            "SIM-MAGV-0001": [Position(182, 434, -16), Position(190, 434, -16), Position(190, 425, 16)], 
            "SIM-MAGV-0002": [Position(182, 440, -16), Position(190, 440, -16), Position(190, 425, 16)], 
            "SIM-MAGV-0003": [Position(182, 446, -16), Position(190, 446, -16), Position(190, 425, 16)], 
            "SIM-MAGV-0004": [Position(196, 434, -16), Position(190, 434, -16), Position(190, 425, 16)],  
            "SIM-MAGV-0005": [Position(196, 440, -16), Position(190, 440, -16), Position(190, 425, 16)],  
            "SIM-MAGV-0006": [Position(196, 446, -16), Position(190, 446, -16), Position(190, 425, 16)], 
        }
        # 六辆小车的状态 0：航线上没有飞机； 1：航线上有飞机 
        self.car_state = [0, 0, 0, 0, 0, 0]
        # 与小车绑定的飞机航线
        self.car_drone_route = {
            "SIM-MAGV-0001": [Position(184, 434, -66), Position(130, 298, -66), Position(148, 184, -66), Position(146, 186, -39)], 
            "SIM-MAGV-0002": [Position(184, 440, -80), Position(564, 394, -80), Position(564, 394, -21)], 
            "SIM-MAGV-0003": [Position(184, 446, -72), Position(508, 514 ,-72), Position(508, 514 ,-27)], 
            "SIM-MAGV-0004": [Position(196, 434, -100), Position(430, 184, -100), Position(430, 184, -15)],  
            "SIM-MAGV-0005": [Position(196, 440, -115), Position(528, 172, -115), Position(528, 172, -25)],  
            "SIM-MAGV-0006": [Position(196, 446, -90), Position(490, 390, -90), Position(490, 390, -27)], 
        }

        self.init_pos = {
            "SIM-MAGV-0001": Position(184, 434, -16), 
            "SIM-MAGV-0002": Position(184, 440, -16), 
            "SIM-MAGV-0003": Position(184, 446, -16), 
            "SIM-MAGV-0004": Position(196, 434, -16),  
            "SIM-MAGV-0005": Position(196, 440, -16),  
            "SIM-MAGV-0006": Position(196, 446, -16), 
        }


    def move_car_with_route(self, car_sn, route, time_est):
        """小车需要设置偏航角yaw"""
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        # if int(car_sn[-1]) <=3:
        #     msg.car_route_info.yaw = 3
        # else:
        #     msg.car_route_info.yaw = 0
        msg.car_route_info.yaw = -1.57
        msg.car_route_info.way_point = route
        self.cmd_pub.publish(msg)
        rospy.loginfo(f'--{car_sn} is prepare for back --')
        rospy.sleep(time_est)

    
    def inspect_car(self, cars):
        # 1 --> 6
        # 需要增加一个条件，其他小车不能动且状态为1
        car_sum = [1, 1, 1, 1, 1, 1]
        for car in cars:
            car_sn = car.sn
            carId = int(car_sn[-1]) - 1
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            if car_work_state != 1:
                self.state = WorkState.STOP_CAR
                # break
            # 如果最后一辆车是这个就会变成RUNNING_CAR
            if car_work_state == 1 and self.des_pos_reached(self.init_pos[car_sn], car_pos, 0.5):
                car_sum[carId] = 1
            else:
                car_sum[carId] = 0

        if sum(car_sum) == 6:
            self.state = WorkState.RUNNING_CAR
        else:
            self.state = WorkState.STOP_CAR
        # 每辆车都处于可运行状态
        
        if self.state == WorkState.RUNNING_CAR:
            for car in cars:
                car_sn = car.sn
                car_work_state = car.car_work_state
                car_pos = car.pos.position
                car_drone_sn = car.drone_sn
                carId = int(car_sn[-1]) - 1

                if self.car_state[carId] == 0:
                    # 去上货点，同时接货 
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 5.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    break

                elif self.car_state[carId] == 1 and car_drone_sn != '':
                    # 起飞无人机
                    self.fly_one_route(car_drone_sn, self.car_drone_route[car_sn], 10, 10, None)
                    self.car_state[carId] = 2
                    self.state == WorkState.STOP_CAR
                    break

                
                elif self.car_state[carId] == 2 and car_drone_sn != '':
                    # 返回上货点
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 5.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    break


                
        



    def main(self):
        rospy.sleep(2.0)
        self.sys_init()
        self.state = WorkState.RUNNING_CAR
        while not rospy.is_shutdown():
            self.inspect_car(self.car_physical_status)
            rospy.sleep(0.04)



if __name__ == "__main__":
    rospy.loginfo("--CarNode is running--")
    car = CarNode()
    car.main()
