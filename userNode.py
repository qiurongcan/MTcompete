
import rospy
import numpy as np
import time
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from user_pkg.msg import UserCmdRequest


"""
创建一个用于放置飞机、更换飞机、上货、充电的节点 具有以下功能
1. 如果车上没有飞机则放置飞机 然后上货 上货后小车返回接机点
2. 如果小车上有飞机 先判断小车上飞机的电量： 如果电量低于30% 则进行充电后上货; 否则直接上货 然后返回接机点
3. 《更换飞机作为备选项》 
"""

class UserNode(DemoPipeline):
    def __init__(self, node_name='qrcUserNode', show=False):
        super().__init__(node_name, show)
        
        """
        car     cargo
        1  -->  1
        2  -->  5
        3  -->  4
        4  -->  2
        5  -->  3
        6  -->  6
        """

        # 【决赛】上货点不变
        self.load_pos = Position(190, 425, -16)

        # 存储和小车绑定的货物索引
        self.cargo_id = {}
        
        self.car_route = {
            "SIM-MAGV-0001": [Position(191, 431, -16), Position(190, 425, -16)], # 直线
            "SIM-MAGV-0002": [Position(188, 438, -16), Position(183, 433, -16), Position(190, 425, -16)], # 折线
            "SIM-MAGV-0003": [Position(182, 440, -16), Position(183, 433, -16), Position(190, 425, -16)], # 折线
            "SIM-MAGV-0004": [Position(198, 432, -16), Position(190, 425, -16)], # 直线 

            "SIM-MAGV-0005": [Position(190, 444, -16), Position(190, 440, -16), Position(190, 425, -16)],  
            "SIM-MAGV-0006": [Position(197, 446, -16), Position(190, 446, -16), Position(190, 425, -16)], 
        }

        # 【决赛】TODO 小车返回的路线及目标点需要改变, 小车5 6 不动
        self.car_back_route = {
            "car1": [Position(190, 425, -16), Position(191, 431, -16)], # 修改
            "car2": [Position(190, 425, -16), Position(183, 433, -16), Position(188, 438, -16)], # 修改
            "car3": [Position(190, 425, -16), Position(183, 433, -16), Position(182, 440, -16)],
            "car4": [Position(190, 425, -16), Position(198, 432, -16)],# 修改

            "car5": [Position(190, 425, -16), Position(190, 440, -16), Position(196, 440, -16)],
            "car6": [Position(190, 425, -16), Position(190, 446, -16), Position(196, 446, -16)],
        }

        self.car_sn_idx = {
            "SIM-MAGV-0001": 'car1', 
            "SIM-MAGV-0002": 'car2', 
            "SIM-MAGV-0003": 'car3', 
            "SIM-MAGV-0004": 'car4', 
            "SIM-MAGV-0005": 'car5', 
            "SIM-MAGV-0006": 'car6', 
        }

        # 存储已经被选择过的订单索引，之后不会在选择
        self.close_cargo_idx = []
    

    def change_pos_to_str(self, bill):
        # change bill's pos to str
        x = bill.target_pos.x
        y = bill.target_pos.y
        z = bill.target_pos.z
        pos = f'{x}|{y}|{z}'
        return pos

    def get_bills_msg(self, bills):
        """
        uint64 index
        Position target_pos
        --bills's status--:
        BILL_UNKOWN = 0
        NOT_STARTED = 1
        DELIVERY = 2
        OVER = 3
        BILL_ERROR = 5
        """
        num_bills = len(bills)
        wait_bills = []

        # rospy.loginfo(bills[0].status)

        sort_bills = {}
        # waiting for delivery
        for bill in bills:
            if bill.status == 1:
                wait_bills.append(bill)
            
            # split bills by different target_pos
            pos = self.change_pos_to_str(bill)
            # 【决赛】存储整个订单的信息，不单单只有索引
            if pos in sort_bills:
                # sort_bills[pos].append(bill.index)
                sort_bills[pos].append(bill)
            else:
                # sort_bills[pos] = [bill.index]
                sort_bills[pos] = [bill]
        return sort_bills

    def move_car_with_route(self, car_sn, route, time_est):
        """小车需要设置偏航角yaw"""
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        # 【决赛不设置偏航角】
        # if int(car_sn[-1]) == 1:
        #     msg.car_route_info.yaw = 2.3
        # elif int(car_sn[-1]) == 4:
        #     msg.car_route_info.yaw = 0.7
        # elif int(car_sn[-1]) <=3 and int(car_sn[-1]) > 1:
        #     msg.car_route_info.yaw = 3
        # else:
        #     msg.car_route_info.yaw = 0
        msg.car_route_info.way_point = route
        self.cmd_pub.publish(msg)
        rospy.loginfo(f'--{car_sn} is prepare for back --')
        rospy.sleep(time_est)


    # TODO 代码存在BUG， 四号小车上不了飞机, 订单的问题，有时候一个订单都没有
    def inspect_user(self, cars):
        for car in cars:
            car_sn = car.sn
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            car_drone_sn = car.drone_sn
            # 小车到达目的地且为准备状态
            if self.des_pos_reached(self.load_pos, car_pos, 0.5) and car_work_state == 1:
                
                # 来上货点的车是空车
                if car_drone_sn == '':
                    # 放置飞机
                    car_drone_sn = self.drone_sn_list[0]
                    print(car_drone_sn)
                    self.move_drone_on_car(car_sn, car_drone_sn, 3.0, WorkState.MOVE_CARGO_IN_DRONE)
                    # 放置后的飞机删除即可
                    self.drone_sn_list.pop(0)
                else:
                    # 来的这辆车带飞机则先查找这辆飞机是哪个
                    for drone in self.drone_physical_status:
                        # 找到这辆飞机
                        if car_drone_sn == drone.sn:
                            # 如果电量低则充电
                            if drone.remaining_capacity <= 30:
                                self.battery_replacement(car_drone_sn, 11.2, WorkState.MOVE_CARGO_IN_DRONE)
                            else:
                                self.state = WorkState.MOVE_CARGO_IN_DRONE
                            break

                # 货物索引
                idx = self.car_sn_idx[car_sn]
                # TODO 在此处需要加入选择订单的算法
                # 【决赛】
                open_cargo = []
                for cargo in self.cargo_id[idx]:
                    current_time = time.time()
                    # 1.订单还未出现或者大概率送不到的时候
                    # 需要将[ms] 转化为[s]
                    if current_time - (cargo.orderTime/1000) < 0 or current_time - (cargo.timeout/1000) >= -150:
                        continue
                    # 2.订单被送过
                    if cargo.index in self.close_cargo_idx:
                        continue
                    open_cargo.append(cargo)
                
                current_time = time.time()
                # TODO 排序无效 修改排序算法
                # sorted_open_cargo = sorted(open_cargo, key=lambda x:(current_time - (x.orderTime/1000)))

                if len(open_cargo) != 0:

                    # 选择排序后的第一个订单    
                    cargo_id0 = open_cargo[0].index
                    
                    # 增加一个送过的货物
                    self.close_cargo_idx.append(cargo_id0)
                    # cargo_id0 = self.cargo_id[idx][0]
                    
                    # TODO 这里可能存在的问题：
                    # sorted_open_cargo 可能为空，为空则无法上货，需要设计一个解决方案

                    self.move_cargo_in_drone(cargo_id0, car_drone_sn, 10.8)
                    # 删除第一个送出去的货物
                    rospy.loginfo(f"Cargo index [{cargo_id0}] is put on {car_drone_sn}")
                    
                    # self.cargo_id[idx].pop(0)
                    car_route = self.car_back_route[idx]
                    # TODO 这个函数需要完善
                    self.move_car_with_route(car_sn, car_route, 3.0)
                else:
                    car_route = self.car_back_route[idx]
                    # TODO 这个函数需要完善
                    self.move_car_with_route(car_sn, car_route, 3.0)






    def main(self):
        rospy.sleep(3.0)
        self.sys_init()
        sort_bills = self.get_bills_msg(self.bills_status)
        # 对订单进行分类
        for key, value in sort_bills.items():
            if key == "146.0|186.0|-34.0":
                self.cargo_id['car1'] = value
            elif key == "564.0|394.0|-16.0":
                self.cargo_id['car5'] =value
            elif key == "508.0|514.0|-22.0":
                self.cargo_id['car3'] =value
            elif key == '430.0|184.0|-10.0':
                self.cargo_id['car4'] =value
            elif key == '528.0|172.0|-20.0':
                self.cargo_id['car6'] =value
            elif key == '490.0|390.0|-22.0':
                self.cargo_id['car2'] =value
        self.state = WorkState.WAIT_CAR
        while not rospy.is_shutdown():
            # 监控小车状态
            self.inspect_user(self.car_physical_status)
            rospy.sleep(0.04)


if __name__ == "__main__":
    rospy.loginfo("----UserNode is running----")
    user = UserNode()
    user.main()
