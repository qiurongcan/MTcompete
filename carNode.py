

import rospy
import numpy as np
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from user_pkg.msg import UserCmdRequest
from std_msgs.msg import Int32, Int32MultiArray, String

"""
创建一个专门控制小车的节点 具有以下功能：
1.初始状态靠边
2.初始开始去接小车
3.航线上有飞机则等待飞机
4.航线上没有飞机且自身没有飞机，则前往上货点装飞机和上货
5.航线上没有飞机但是自身有飞机，则前往上货点上货
6.航线上没有飞机，但自身有飞机，且飞机上有货物，则放飞飞机，然后等待飞机
"""

"""
TODO
1.身上有货物的优先起飞 Finished
2.缩短一些没必要的时间 Finished
3.小车的调度速度可以更快 +1
4.需要增加一个自救措施 直接扩大检测范围即可 Finished
5.增加一个队列判断方法 
    1和4小车在上货点离开时 后续的小车都可以马上运行
    2 和 5 小车离开时， 2和5到一定位置时 1和4 可以动 3和6 不能动
    3 和 6 离开某一个位置a时 1和4可以动
    3 和 6 离开某一个位置b时 2和5可以动
6.可能需要调整配送路线
"""

"""
TODO 【决赛】
1. 小车的布局
2. 
"""

class CarNode(DemoPipeline):
    def __init__(self, node_name='qrcCarNode', show=False):
        super().__init__(node_name, show)

        """
        car     cargo
        1(184, 434, -16)  -->  1 (146, 186, -34)    z = -66         d = 250 + 132
        2(184, 440, -16)  -->  5 (564, 394, -16)    z = -80  直线   d = 382 + 160
        3(184, 446, -16)  -->  4 (508, 514, -22)    z = -72  直线   d = 331 + 144
        4(196, 434, -16)  -->  2 (430, 184, -10)    z = -100 直线   d = 342 + 200
        5(196, 440, -16)  -->  3 (528, 172, -20)    z = -115 直线   d = 426 + 230
        6(196, 446, -16)  -->  6 (490, 390, -22)    z = -90 直线    d = 299 + 180
        按道理6 号小车的优先级应该比 5 号小车的优先级高
        可以让 1 2 4 5 小车送距离最短的货物 3 6 送距离长的 或许又能提高一些时间
        1 - 1
        2 - 6 -90
        3 - 4 -72
        4 - 2 -100
        5 - 5 -80
        6 - 3 -115
        """
        # 动态运动的标志
        self.flag = 0
        
        # 订阅动态运动的标志
        self.flag_sub = rospy.Subscriber("/run_flag", Int32, self.flag_cb)

        # 3号和5号小车离开边界则马上起飞后，小车需要移动的路线
        self.two_fly_car_route = [Position(181, 431, -16), Position(186, 435, -16), Position(186, 438, -16)]
        self.three_fly_car_route = [Position(181, 431, -16), Position(181, 440, -16)]
        
        self.five_fly_car_route = [Position(199, 431, -16), Position(195, 437, -16), Position(194, 438, -16)]
        self.six_fly_car_route = [Position(199, 431, -16), Position(199, 440, -16)]
        
        # ----------------------初始化移动的位置---------------------- # 
        # 一号小车先腾出一个位置
        self.one_car_init_route = [Position(183, 434, -16), Position(183, 431, -16), Position(187, 431, -16)]
        # 二号小车腾出一个位置
        self.two_car_init_route = [Position(190, 438, -16), Position(186, 438, -16)]
        # 三号小车腾出一个位置
        self.three_car_init_route = [Position(183, 446, -16), Position(181, 431, -16)]
        # 四号小车腾出一个位置
        self.four_car_init_route = [Position(197, 434, -16), Position(198, 431, -16), Position(193, 431, -16)]
        # 五号小车腾出一个位置
        self.five_car_init_route = [Position(190, 444, -16), Position(194, 438, -16)]
        # 六号小车腾出一个位置
        self.six_car_init_route = [Position(197, 446, 16), Position(199, 445, -16), Position(199, 431, -16)]
        # ---------------------------------------------------------- #

        # TODO【决赛】6辆小车都需要修改  1 和 4 之间或许可以用直线
        self.car_route = {
            "SIM-MAGV-0001": [Position(187, 431, -16), Position(190, 425, -16)], # 1
            "SIM-MAGV-0002": [Position(181, 431, -16), Position(185, 425, -16), Position(190, 425, -16)], # 2
            "SIM-MAGV-0003": [Position(181, 431, -16), Position(185, 425, -16), Position(190, 425, -16)], # 3
            "SIM-MAGV-0004": [Position(193, 431, -16), Position(190, 425, -16)], # 4
            "SIM-MAGV-0005": [Position(199, 431, -16), Position(195, 425, -16), Position(190, 425, -16)],  
            "SIM-MAGV-0006": [Position(199, 431, -16), Position(195, 425, -16), Position(190, 425, -16)], 
        }
        
        # 六辆小车的状态 0：航线上没有飞机； 1：航线上有飞机 
        self.car_state = [0, 0, 0, 0, 0, 0]
        # 创建一个小车状态的发布者
        self.car_state_pub = rospy.Publisher("/car_state", Int32MultiArray, queue_size=10)
        self.state_msg = Int32MultiArray()
        # 10.25需要修改航线
        self.car_drone_route = {
            "SIM-MAGV-0001": [Position(187, 431, -73), Position(430, 184, -73), Position(430, 184, -14)],
            "SIM-MAGV-0002": [Position(181, 431, -79), Position(490, 390, -79), Position(490, 390, -26)], #
            "SIM-MAGV-0003": [Position(181, 431, -93), Position(508, 514 ,-93), Position(508, 514 ,-26)], ###
            "SIM-MAGV-0004": [Position(193, 431, -66), Position(130, 298, -66), Position(148, 184, -66), Position(146, 186, -38)], 
            "SIM-MAGV-0005": [Position(199, 431, -85), Position(564, 394, -85), Position(564, 394, -20)], ### 
            "SIM-MAGV-0006": [Position(199, 431, -115), Position(528, 172, -115), Position(528, 172, -24)],
        }

        # 【决赛】修改所有小车的初始位置 6号小车不动
        self.init_pos = {
            "SIM-MAGV-0001": Position(187, 431, -16), 
            "SIM-MAGV-0002": Position(181, 431, -16), 
            "SIM-MAGV-0003": Position(181, 431, -16), # 3号小车提前起飞

            "SIM-MAGV-0004": Position(193, 431, -16), 
            "SIM-MAGV-0005": Position(199, 431, -16), # 5号小车提前起飞
            "SIM-MAGV-0006": Position(199, 431, -16), 
        }
        
        # 【决赛】修改
        self.work_init_pos = {
            "SIM-MAGV-0001": Position(187, 431, -16),  # 修改
            "SIM-MAGV-0002": Position(186, 438, -16),  # 修改
            "SIM-MAGV-0003": Position(181, 440, -16),  # 修改
            "SIM-MAGV-0004": Position(193, 431, -16),  # 修改
            "SIM-MAGV-0005": Position(194, 438, -16),  # 修改
            "SIM-MAGV-0006": Position(199, 440, -16),  # 修改
        }

        self.load_pos = Position(190, 425, -16)

        self.flag_4_init = 1


    def flag_cb(self, msg):
        self.flag = msg.data

    def sys_init(self):
        rospy.sleep(10.0)
        self.state = WorkState.TEST_MAP_QUERY

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

  
    # 先完成静态的
    def inspect_car(self, cars):
        
        # 需要增加一个条件，其他小车不能动且状态为1
        car_sum = [0, 0, 0, 0, 0, 0]
        for car in cars:
            car_sn = car.sn
            carId = int(car_sn[-1]) - 1
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            # 【决赛】所有小车位于起始位置
            if car_work_state == 1:
                if self.des_pos_reached(self.init_pos[car_sn], car_pos, 1.0):
                    car_sum[carId] = 1
                # 【决赛修改】
                elif self.des_pos_reached(self.work_init_pos[car_sn], car_pos, 1.0):
                    car_sum[carId] = 1


        if sum(car_sum) == 6 or self.flag == 1:
            self.state = WorkState.RUNNING_CAR
        else:
            self.state = WorkState.STOP_CAR
        # 每辆车都处于可运行状态
        

        # 优先检查身上有飞机的且有货物的，然后起飞
        if self.state == WorkState.RUNNING_CAR:
            idx = 0
            # 将身上有飞机的队列优先
            for car in cars:
                car_sn = car.sn
                car_work_state = car.car_work_state
                car_pos = car.pos.position
                car_drone_sn = car.drone_sn
                carId = int(car_sn[-1]) - 1
                # 如果都没有则正常执行
                if self.car_state[carId] == 1 and car_drone_sn != '':
                    cars.pop(idx)
                    cars.insert(0, car)
                    break
                idx += 1

            

            for car in cars:
                car_sn = car.sn
                car_work_state = car.car_work_state
                
                if car_work_state != 1:
                    continue
                car_pos = car.pos.position
                car_drone_sn = car.drone_sn
                carId = int(car_sn[-1]) - 1

                if self.car_state[carId] == 1 and car_drone_sn != '':
                    # 起飞无人机 缩短为1秒
                    self.fly_one_route(car_drone_sn, self.car_drone_route[car_sn], 10, 3.0, None)
                    # 对于3车和5车而言，飞机起飞后需要挪动小车
                    if carId == 1:
                        self.move_car_with_route("SIM-MAGV-0002", self.two_fly_car_route, 1.0)
                        print("--------------------")
                    elif carId == 2:
                        self.move_car_with_route("SIM-MAGV-0003", self.three_fly_car_route, 1.0)
                        # print("--------------------")
                    elif carId == 4:
                        self.move_car_with_route("SIM-MAGV-0005", self.five_fly_car_route, 1.0)
                        print("-------------------")
                    elif carId == 5:
                        self.move_car_with_route("SIM-MAGV-0006", self.six_fly_car_route, 1.0)
                        # print("--------------------")
                    self.car_state[carId] = 2
                    self.state == WorkState.STOP_CAR
                    break

                # 初始状态的小车
                if self.car_state[carId] == 0 and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2.0):
                    # 4 号小车第一次跳过
                    if carId == 3 and self.flag_4_init:
                        if self.car_state[4] == 1:
                            self.flag_4_init = 0
                        continue
                    # 去上货点，同时接货 
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 2.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    self.flag = 0
                    break
                
                elif self.car_state[carId] == 2 and car_drone_sn != '' and self.des_pos_reached(car_pos, self.init_pos[car_sn], 2.0):
                    # 返回上货点
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 2.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    self.flag = 0
                    break


        # -----------对需要起飞的无人机进行起飞------------
        for car in cars:
            car_sn = car.sn
            car_work_state = car.car_work_state
            if car_work_state != 1:
                continue
            car_pos = car.pos.position
            car_drone_sn = car.drone_sn
            carId = int(car_sn[-1]) - 1
            # 检查一下有没有需要起飞的无人机
            if self.car_state[carId] == 1 and car_drone_sn != '' and self.des_pos_reached(self.init_pos[car_sn], car_pos, 1.0):
                # 起飞无人机 缩短为1秒
                for drone in self.drone_physical_status:
                    if car_drone_sn == drone.sn:
                        if drone.bind_cargo_id != 0:
                            print("PRE TAKEOFF")
                            self.fly_one_route(car_drone_sn, self.car_drone_route[car_sn], 10, 2.0, None)
                            if carId == 1:
                                self.move_car_with_route("SIM-MAGV-0002", self.two_fly_car_route, 2.0)
                                # print("----------------")
                            elif carId == 2:
                                self.move_car_with_route("SIM-MAGV-0003", self.three_fly_car_route, 2.0)
                            elif carId == 4:
                                self.move_car_with_route("SIM-MAGV-0005", self.five_fly_car_route, 2.0)
                                # print("================")
                            elif carId == 5:
                                self.move_car_with_route("SIM-MAGV-0006", self.six_fly_car_route, 2.0)
                            self.car_state[carId] = 2
                            break
                self.state == WorkState.STOP_CAR
                break
        
        # 将小车的状态发布出去
        self.state_msg.data = self.car_state
        self.car_state_pub.publish(self.state_msg)
        

    # 对小车的运行顺序进行排序
    def sort_car_msg(self, cars):
        # 直接使用填充的方式进行排序
        process_cars = [None, None, None, None, None, None]
        # 【决赛】小车排序 [1, 6, 3, 4, 2, 5]
        for car in cars:
            car_sn = car.sn
            carId = int(car_sn[-1])
            if carId == 1 : # 1号小车
                process_cars[0] = car
            elif carId == 6: #
                process_cars[1] = car
            elif carId == 3:
                process_cars[2] = car
            elif carId == 4:
                process_cars[3] = car
            elif carId == 2:
                process_cars[4] = car
            elif carId == 5:
                process_cars[5] = car
        return process_cars
            



    def main(self):
        # rospy.sleep(10.0)
        self.sys_init()
        self.state = WorkState.RUNNING_CAR
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
            # 设置优先顺序
            car_msgs = self.sort_car_msg(self.car_physical_status)
            self.inspect_car(car_msgs)
            



if __name__ == "__main__":
    rospy.loginfo("--CarNode is running--")
    car = CarNode()
    car.main()
