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


        self.route_2 = [Position(186, 438, -16), Position(186, 435, -16), Position(181, 431, -16), Position(184, 425, -16)] # 2号小车的路线
        self.route_3 = [Position(181, 440, -16), Position(181, 431, -16), Position(184, 425, -16)] # 3号小车路线
        self.route_5 = [Position(194, 438, -16), Position(194, 435, -16), Position(199, 435, -16), Position(199, 431, -16), Position(196, 424, -16)] # 5号小车路线  
        self.route_6 = [Position(199, 440, -16), Position(199, 431, -16), Position(196, 424, -16)] # 6号小车路线

        
        self.init_pos = {
            "SIM-MAGV-0002": Position(186, 438, -16), 
            "SIM-MAGV-0003": Position(181, 440, -16), 
            "SIM-MAGV-0005": Position(194, 438, -16), 
            "SIM-MAGV-0006": Position(199, 440, -16), 
        }
        # 用于发布离开的小车的顺序
        self.pub_car_order = rospy.Publisher("/car_order", Int32, queue_size=10)

        self.move_2 = False
        self.move_3 = False

        self.move_5 = False
        self.move_6 = False

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

                # 2车和3车是等效的
                # 2号小车
                if carId == 1 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 如果是2号小车 判断2号小车要什么时候才能动

                    
                    if car_drone_sn != '' and self.move_3 == False:
                        self.move_car_with_route(car_sn, self.route_2, 2.0)
                        # 发布2号小车离开的消息
                        self.pub_car_order.publish(2)
                        self.move_2 = True
                    
                    elif car_drone_sn == "":
                        self.move_2 = False
                
                if carId == 2 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2):
                    if car_drone_sn != '' and self.move_2 == False:
                        self.move_car_with_route(car_sn, self.route_3, 2.0)
                        # 发布3号小车离开的消息
                        self.pub_car_order.publish(3)
                        self.move_3 = True
                    
                    elif car_drone_sn == '':
                        self.move_3 = False

                
                elif carId == 4 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 如果是2号小车 判断2号小车要什么时候才能动

                    
                    if car_drone_sn != '' and self.move_6 == False:
                        self.move_car_with_route(car_sn, self.route_5, 2.0)
                        # 发布5号小车离开的消息
                        self.pub_car_order.publish(5)
                        self.move_5 = True
                    
                    elif car_drone_sn == "":
                        self.move_5 = False

                elif carId == 5 and car_work_state == 1 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2): # 如果是2号小车 判断2号小车要什么时候才能动

                    
                    if car_drone_sn != '' and self.move_5 == False:
                        self.move_car_with_route(car_sn, self.route_6, 2.0)
                        # 发布6号小车离开的消息
                        self.pub_car_order.publish(6)
                        self.move_6 = True
                    
                    elif car_drone_sn == "":
                        self.move_6 = False
                    
                    
                   


if __name__ == "__main__":
    # 运行主函数
    pos = PosCar()
    pos.main()


