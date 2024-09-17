
import rospy
import numpy as np
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

        # 上货点
        self.load_pos = Position(190, 425, -16)

        # 存储和小车绑定的货物索引
        self.cargo_id = {}
        
        
        self.car_back_route = {
            "car1": [Position(190, 425, -16), Position(190, 434, -16), Position(184, 434, -16)],
            "car2": [Position(190, 425, -16), Position(190, 440, -16), Position(184, 440, -16)],
            "car3": [Position(190, 425, -16), Position(190, 446, -16), Position(184, 446, -16)],
            "car4": [Position(190, 425, -16), Position(190, 434, -16), Position(196, 434, -16)],
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
            if pos in sort_bills:
                sort_bills[pos].append(bill.index)
            else:
                sort_bills[pos] = [bill.index]
        return sort_bills

    def move_car_with_route(self, car_sn, route, time_est):
        """小车需要设置偏航角yaw"""
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        if int(car_sn[-1]) <=3:
            msg.car_route_info.yaw = 3
        else:
            msg.car_route_info.yaw = 0
        msg.car_route_info.way_point = route
        self.cmd_pub.publish(msg)
        rospy.loginfo(f'--{car_sn} is prepare for back --')
        rospy.sleep(time_est)


    def inspect_user(self, cars):
        for car in cars:
            car_sn = car.sn
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            car_drone_sn = car.drone_sn
            if self.des_pos_reached(self.load_pos, car_pos, 0.5) and car_work_state == 1:
                
                if car_drone_sn == '':
                    # 放置飞机
                    car_drone_sn = self.drone_sn_list[0]
                    print(car_drone_sn)
                    self.move_drone_on_car(car_sn, car_drone_sn, 3.0, WorkState.MOVE_CARGO_IN_DRONE)
                    # 放置后的飞机删除即可
                    self.drone_sn_list.pop(0)
                else:
                    for drone in self.drone_physical_status:
                        if car_drone_sn == drone.sn:
                            if drone.remaining_capacity <= 30:
                                self.battery_replacement(car_drone_sn, 11, WorkState.MOVE_CARGO_IN_DRONE)
                            else:
                                self.state = WorkState.MOVE_CARGO_IN_DRONE
                            
                            break
                # 货物索引
                idx = self.car_sn_idx[car_sn]
                cargo_id0 = self.cargo_id[idx][0]
                self.move_cargo_in_drone(cargo_id0, car_drone_sn, 11, WorkState.MOVE_CAR_GO_TO_DROPOFF)
                # 删除第一个送出去的货物
                self.cargo_id[idx].pop(0)
                car_route = self.car_back_route[idx]
                # TODO 这个函数需要完善
                self.move_car_with_route(car_sn, car_route, 3.0)





    def main(self):
        rospy.sleep(3.0)
        self.sys_init()
        sort_bills = self.get_bills_msg(self.bills_status)
        for key, value in sort_bills.items():
            if key == "146.0|186.0|-34.0":
                self.cargo_id['car1'] = value
            elif key == "564.0|394.0|-16.0":
                self.cargo_id['car2'] =value
            elif key == "508.0|514.0|-22.0":
                self.cargo_id['car3'] =value
            elif key == '430.0|184.0|-10.0':
                self.cargo_id['car4'] =value
            elif key == '528.0|172.0|-20.0':
                self.cargo_id['car5'] =value
            elif key == '490.0|390.0|-22.0':
                self.cargo_id['car6'] =value
        self.state = WorkState.WAIT_CAR
        while not rospy.is_shutdown():
            self.inspect_user(self.car_physical_status)
            rospy.sleep(0.04)


if __name__ == "__main__":
    rospy.loginfo("----UserNode is running----")
    user = UserNode()
    user.main()
