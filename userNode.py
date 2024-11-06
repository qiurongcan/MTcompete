
import rospy
import numpy as np
import time
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from user_pkg.msg import UserCmdRequest
from std_msgs.msg import Int32, Int32MultiArray


"""
TODO 解决连续送一个目的地的订单
"""

class UserNode(DemoPipeline):
    def __init__(self, node_name='qrcUserNode', show=False):
        super().__init__(node_name, show)

        # 创建一个小车离开上货点的发布者
        self.flag_pub = rospy.Publisher("/recycle_flag", Int32, queue_size=10)
        self.flag = 1 # 其余小车可以来
        self.stop = 0 # 禁止来

        self.unlock_pub = rospy.Publisher("/unlock", Int32, queue_size=10)

        # 【决赛】上货点不变
        self.load_pos = Position(190, 425, -16)

        self.drone_back_pub = rospy.Publisher("/drone_back", Int32MultiArray, queue_size=10)

        self.back_data = [0, 0, 0, 0, 0, 0]
        # self.drone_back_sub = rospy.Subscriber("/drone_back2", Int32MultiArray, self.drone_back_cb)

        
        # 【决赛】TODO 小车的返回路线 6辆小车的路线都需要修改
        self.car_back_route = {
            "SIM-MAGV-0001": [Position(190, 425, -16), Position(187, 426, -16), Position(187, 431, -16)], # 
            "SIM-MAGV-0002": [Position(190, 425, -16), Position(184, 425, -16), Position(181, 431, -16), Position(186, 438, -16)], #
            "SIM-MAGV-0003": [Position(190, 425, -16), Position(184, 425, -16), Position(181, 431, -16), Position(181, 440, -16)],
            "SIM-MAGV-0004": [Position(190, 425, -16), Position(193, 431, -16)], #
            "SIM-MAGV-0005": [Position(190, 425, -16), Position(196, 424, -16), Position(199, 431, -16), Position(199, 434, -16), Position(194, 438, -16)],
            "SIM-MAGV-0006": [Position(190, 425, -16), Position(196, 424, -16), Position(199, 431, -16), Position(199, 440, -16)],# 
        }


        # 存储已经被选择过的订单索引，之后不会在选择
        self.close_cargo_idx = []

        self.back_lock = [0, 0, 0, 0, 0, 0]
        self.back_lock_sub = rospy.Subscriber("/back_lock_pub", Int32MultiArray, self.back_lock_cb)
        self.back_lock_pub = rospy.Publisher("/back_lock", Int32MultiArray, queue_size=10)
        self.msg = Int32MultiArray()

        # 发送给小车节点的，用于破除循环锁
        self.move_flag_pub = rospy.Publisher("/move_flag", Int32, queue_size=10)

    def back_lock_cb(self, msg):
        self.back_lock = list(msg.data)

    # def drone_back_cb(self, msg):
    #     self.back_data = list(msg.data)
    
    def move_car_with_route(self, car_sn, route, time_est):
        """现在的容器不设置偏航角也可以移动"""
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point = route
        self.cmd_pub.publish(msg)
        rospy.loginfo(f'--{car_sn} is prepare for back --')
        rospy.sleep(time_est)

    def inspect_user2(self):
        # 1号小车负责上货，其余小车负责放回飞机
        for car in self.car_physical_status:
            car_sn = car.sn
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            car_drone_sn = car.drone_sn
            carId = int(car_sn[-1]) - 1
            if self.des_pos_reached(self.load_pos, car_pos, 0.5) and car_work_state == 1:
                # 4号小车
                if carId == 3:
                    if car_drone_sn == '':
                        self.move_drone_on_car(car_sn, self.drone_sn_list[0], 2.8, None)
                        car_drone_sn = self.drone_sn_list[0]
                        self.drone_sn_list.pop(0)
                    
                    
                    
                    open_cargo = []
                    # 跳过2 4 5订单
                    # TODO 增加白嫖订单的方法
                    for cargo in self.bills_status:
                        if int(cargo.index % 6) == 2:
                            continue
                        else:
                            current_time = time.time()
                            orderTime = cargo.orderTime
                            if (current_time - (orderTime/1000)) < 0 or (current_time -(orderTime/1000))> 80:
                                continue
                            else:
                                open_cargo.insert(0, cargo)
                    
                    if len(open_cargo) == 0:
                        break

                    else:
                        # 首先选一个货物索引
                        for cargo_rev in open_cargo:
                            cargo_idx = cargo_rev.index
                            if cargo_idx in self.close_cargo_idx:
                                cargo_idx = -1
                                continue
                            else:
                                # 如果只有一单呢？
                                temp_idx = int(cargo_idx % 6)
                                if self.back_data[temp_idx] >= 2:
                                    cargo_idx = -1
                                    continue
                                else:
                                    break
                        
                        # 重新等待
                        if cargo_idx == -1:
                            if sum(self.back_data) == 6 and sum(self.back_lock) == 3:
                                self.move_car_with_route(car_sn, self.car_back_route[car_sn], 3.5)
                                self.flag_pub.publish(self.flag)
                                rospy.sleep(1.5)
                                self.flag_pub.publish(self.stop)
                                # 小车状态重置 发布0？
                                self.move_flag_pub.publish(0)
                            break
                        
                        rospy.loginfo(f'--------{cargo_idx}--------')
                        self.close_cargo_idx.append(cargo_idx)
                        self.move_cargo_in_drone(cargo_idx, car_drone_sn, 10.8)
                        
                        # 通信节点
                        temp_id = int(cargo_idx % 6)
                        self.back_data[temp_id] += 1

                        # 1号小车移动返回               
                        self.move_car_with_route(car_sn, self.car_back_route[car_sn], 4.0)
                        # 小车离开后，表示不能返回
                        # --------------------------------------------------------------
                        # 只有1.5s的窗口期
                        self.flag_pub.publish(self.flag)
                        rospy.sleep(1.5)
                        self.flag_pub.publish(self.stop)
                
                # 其余小车
                else:
                    # pass
                    if car_drone_sn != '':
                        self.drone_retrieve(car_drone_sn, car_sn, 0.5, None)
                        # TODO 可以简化一下这段代码
                        if carId == 5: # 6号小车
                            self.back_data[1] += -1
                            self.back_lock[1] += -1

                        elif carId == 4: # 5号小车
                            self.back_data[4] += -1
                            self.back_lock[4] += -1
                            pass

                        elif carId == 1: # 2号小车
                            self.back_data[5] += -1
                            self.back_lock[5] += -1

                        elif carId ==2: # 3号小车
                            self.back_data[3] += -1
                            self.back_lock[3] += -1

                        elif carId == 0: # 1号小车
                            self.back_data[0] += -1
                            self.back_lock[0] += -1

                        self.msg.data = self.back_lock
                        self.back_lock_pub.publish(self.msg)

                        # 在小车sn列表的末尾增加放回的飞机的sn
                        self.drone_sn_list.append(car_drone_sn)
                        rospy.loginfo(f"---- {car_drone_sn} is being put back ----")
                        self.move_car_with_route(car_sn, self.car_back_route[car_sn], 2.5)
                        # 对四号小车解锁
                        self.unlock_pub.publish(1)
              
                

    def main(self):
        rospy.sleep(5.0)
        # self.drone_sn_list
        while not rospy.is_shutdown():
            # 监控小车状态
            self.inspect_user2()



if __name__ == "__main__":
    rospy.loginfo("----UserNode is running----")
    user = UserNode()
    user.main()
