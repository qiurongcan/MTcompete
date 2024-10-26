# 创建一个新的节点，主要负责2、3；5、6号小车位置递补

import rospy
import numpy as np
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from user_pkg.msg import UserCmdRequest
from std_msgs.msg import Int32, Int32MultiArray, String

"""
有两个位置
2 3 小车对应 (181, 431, -16) 现在2 3 小车需要轮流递补这个位置
5 6 小车对应 (199, 431, -16) 现在5 6 小车需要轮流递补这个位置
"""

class PosCar(DemoPipeline): # 不知道起什么名字随便起了
    def __init__(self, node_name='qrcPosNode', show=False):
        super().__init__(node_name, show)

        self.car_state = [-1, -1, -1, -1, -1, -1]  # 占位
        self.car_state_sub = rospy.Subscriber('/car_state', Int32MultiArray, self.car_state_cb)
        self.route_2 = [Position(186, 438, -16), Position(186, 435, -16), Position(181, 431, -16)] # 2号小车的路线
        self.route_3 = [Position(181, 440, -16), Position(181, 431, -16)] # 3号小车路线
        self.route_5 = [Position(194, 438, -16), Position(194, 435, -16), Position(199, 435, -16), Position(199, 431, -16)] # 5号小车路线
        self.route_6 = [Position(199, 440, -16), Position(199, 431, -16)] # 6号小车路线

        
        self.init_pos = {
            "SIM-MAGV-0001": Position(187, 431, -16), 
            "SIM-MAGV-0002": Position(186, 438, -16), 
            "SIM-MAGV-0003": Position(181, 440, -16), # 3号小车提前起飞
            "SIM-MAGV-0004": Position(193, 431, -16), 
            "SIM-MAGV-0005": Position(194, 438, -16), # 5号小车提前起飞
            "SIM-MAGV-0006": Position(199, 440, -16), 
        }
    
    # 获取小车状态的回调函数
    def car_state_cb(self, msg):
        self.car_state = msg.data
        self.car_state = list(self.car_state) # 需要转化为列表才是可变的

    def move_car_with_route(self, car_sn, route, time_est):
        """小车需要设置偏航角yaw"""
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point = route
        self.cmd_pub.publish(msg)
        rospy.loginfo(f'--{car_sn} is moving to load cargo --')
        rospy.sleep(time_est)

    
    def get_x_car_msg(self, car_num):
        # 用于获取某辆小车的状态
        # car_num 表示几号小车 范围 [0, 1, 2, 3, 4, 5]
        for car in self.car_physical_status:
            car_sn = car.sn
            carId = int(car_sn[-1]) - 1
            if carId != car_num:
                continue
            # 如果编号符合则返回这辆小车的信息
            return car



    def main(self):
                
        while not rospy.is_shutdown():
            # 监控每一辆小车的状态
            if self.car_physical_status is None:
                continue
            for car in self.car_physical_status:
                car_sn = car.sn
                carId = int(car_sn[-1]) - 1
                # 跳过1 和 4号小车
                if carId == 0 or carId == 3:
                    continue
                
                car_work_state = car.car_work_state
                car_pos = car.pos.position
                car_drone_sn = car.drone_sn

                if carId == 1 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 如果是2号小车 判断2号小车要什么时候才能动
                    car3 = self.get_x_car_msg(2) # 获取3号小车的信息
                    # 状态为0 可以
                    if self.car_state[carId] == 0:
                        if self.car_state[2] == 2 and car3.drone_sn == '':
                            self.move_car_with_route(car_sn, self.route_2, 1.0)
                            
                    
                    # 状态为 2 且身上有飞机可以
                    if self.car_state[carId] == 2 and car_drone_sn != "":
                        if self.car_state[2] == 2 and car3.drone_sn == '':
                            self.move_car_with_route(car_sn, self.route_2, 1.0)
                            

                if carId == 2 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 如果是3号小车 判断3号小车要什么时候才能动
                    car2 = self.get_x_car_msg(1) # 获取2号小车的信息
                    if self.car_state[carId] == 2 and car_drone_sn != '':
                        if self.car_state[1] == 2 or self.car_state == 0:
                            self.move_car_with_route(car_sn, self.route_3, 1.0)
                            

                # 同理 5 和 6 号小车原理是一样的

                if carId == 4 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 如果是5号小车
                    car6 = self.get_x_car_msg(5) # 获取6号小车的信息
                    if self.car_state[carId] == 0:
                        if self.car_state[5] == 2 and car6.drone_sn == '':
                            self.move_car_with_route(car_sn, self.route_5, 1.0)
                            
                    
                    if self.car_state[carId] == 2 and car_drone_sn != '':
                        if self.car_state[5] == 2 and car6.drone_sn == '':
                            self.move_car_with_route(car_sn, self.route_5, 1.0)
                            

                if carId == 5 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 6号小车
                    car5 = self.get_x_car_msg(4)
                    if self.car_state[carId] == 2 and car_drone_sn != '':
                        if self.car_state[4] == 2 or self.car_state[4] == 0:
                            self.move_car_with_route(car_sn, self.route_6, 1.0)
                            


                    


if __name__ == "__main__":
    # 运行主函数
    pos = PosCar()
    pos.main()


