

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
        # 3号和5号小车离开边界则马上起飞后，小车需要移动的路线
        self.three_fly_car_route = [Position(181, 432, -16), Position(181, 440, -16)]
        self.five_fly_car_route = [Position(181, 432, -16), Position(186, 438, -16)]

        # 一号小车先腾出一个位置
        self.one_car_init_route = [Position(183, 434, -16), Position(183, 431, -16), Position(193, 431, -16)]

        # 二号小车腾出一个位置
        self.two_car_init_route = [Position(190, 438, -16), Position(185, 436, -16), Position(185, 431, -16), Position(187, 431, -16)]

        # 三号小车腾出一个位置
        self.three_car_init_route = [Position(183, 446, -16), Position(181, 440, -16)]

        # 四号小车腾出一个位置
        self.four_car_init_route = [Position(197, 434, -16), Position(199, 431, -16)]

        # 五号小车腾出一个位置
        self.five_car_init_route = [Position(190, 444, -16),Position(190, 438, -16), Position(186, 438, -16)]


        # 【决赛】6号小车不动 10.22修改
        self.car_route = {
            "SIM-MAGV-0001": [Position(193, 431, -16), Position(190, 425, -16)], # 直线
            "SIM-MAGV-0002": [Position(187, 431, -16), Position(190, 425, -16)], # 折线
            "SIM-MAGV-0003": [Position(181, 440, -16), Position(181, 427, -16), Position(190, 425, -16)], # 折线
            "SIM-MAGV-0004": [Position(199, 431, -16), Position(199, 426, -16), Position(190, 425, -16)], # 折线 
            "SIM-MAGV-0005": [Position(186, 438, -16), Position(181, 433, -16), Position(181, 427, -16), Position(190, 425, -16)],  

            "SIM-MAGV-0006": [Position(197, 446, -16), Position(190, 446, -16), Position(190, 425, -16)], 
        }
        
        # 六辆小车的状态 0：航线上没有飞机； 1：航线上有飞机 
        self.car_state = [0, 0, 0, 0, 0, 0]
        
        # 【决赛】与小车绑定的飞机航线 6号航线不动 降落距离全部-1
        self.car_drone_route = {
            "SIM-MAGV-0001": [Position(193, 431, -66), Position(130, 298, -66), Position(148, 184, -66), Position(146, 186, -38)], 
            "SIM-MAGV-0002": [Position(187, 431, -80), Position(490, 390, -80), Position(490, 390, -26)],
            "SIM-MAGV-0003": [Position(181, 432, -75), Position(508, 514 ,-75), Position(508, 514 ,-26)], 
            "SIM-MAGV-0004": [Position(199, 431, -70), Position(430, 184, -70), Position(430, 184, -14)],  
            "SIM-MAGV-0005": [Position(181, 432, -85), Position(564, 394, -85), Position(564, 394, -20)], 
            "SIM-MAGV-0006": [Position(197, 446, -115), Position(528, 172, -115), Position(528, 172, -24)],  
        }

        # 【决赛】修改所有小车的初始位置 5和6号小车不动
        self.init_pos = {
            "SIM-MAGV-0001": Position(183, 434, -16), 
            "SIM-MAGV-0002": Position(190, 438, -16), 
            "SIM-MAGV-0003": Position(181, 432, -16), # 3号小车提前起飞
            "SIM-MAGV-0004": Position(197, 434, -16), 
            "SIM-MAGV-0005": Position(181, 432, -16), # 5号小车提前起飞

            "SIM-MAGV-0006": Position(197, 446, -16), 
        }
        
        # 【决赛】修改
        self.work_init_pos = {
            "SIM-MAGV-0001": Position(193, 431, -16),  # 修改
            "SIM-MAGV-0002": Position(187, 431, -16),  # 修改
            "SIM-MAGV-0003": Position(181, 440, -16),  # 修改
            "SIM-MAGV-0004": Position(199, 431, -16),  # 修改
            "SIM-MAGV-0005": Position(186, 438, -16),  # 修改 10.22

            "SIM-MAGV-0006": Position(197, 446, -16), 
        }

        self.load_pos = Position(190, 425, -16)


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

    
    def inspect_car_plus(self, cars):
        """
        2024.9.19
        调度速度更加灵活的版本
        """
        # 二进制编码
        car_sum = [0, 0, 0, 0, 0, 0]
        # 判断各种状态是否可以可以执行下一步
        for car in cars:
            car_sn = car.sn
            carId = int(car_sn[-1]) - 1
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            # 一定是先READY状态然后才非READY状态，非READY状态主要是针对 1 4 和 2 5
            # 先判断所有车的状态是否都为1, 小车1 和 小车4 时必须为READY状态
            # 小车为非READY状态时
            if car_work_state != 1:
                car_drone_sn = car.drone_sn
                if carId == 0 or carId == 3:
                    if self.des_pos_reached(car_pos, self.load_pos, 2.5) == False:
                        if car_drone_sn != '':
                            # print(car_drone_sn)
                            for drone in self.drone_physical_status:
                                # print("-------", drone.sn)
                                # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回
                                if car_drone_sn == drone.sn:
                                    # print(drone)
                                    if drone.bind_cargo_id != 0:
                                        # print("_________+++++++")
                                        car_sum[carId] = 1
                                    break
                
                # 需要查找小车上的飞机是否含有货物
                if carId == 1 or carId == 4:
                    if self.des_pos_reached(car_pos, self.load_pos, 13.0) == False:
                        if car_drone_sn != '':
                            # print(car_drone_sn)
                            for drone in self.drone_physical_status:
                                # print("-------", drone.sn)
                                # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回
                                if car_drone_sn == drone.sn:
                                    # print(drone)
                                    if drone.bind_cargo_id != 0:
                                        # print("--------")
                                        car_sum[carId] = 1
                                    break

                elif carId == 2 or carId == 5:
                    # 加大限制
                    if self.des_pos_reached(car_pos, self.load_pos, 20.5) == False:
                        if car_drone_sn != '':
                            # print(car_drone_sn)
                            for drone in self.drone_physical_status:
                                # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回
                                # print("-------", drone.sn)
                                if car_drone_sn == drone.sn:
                                    if drone.bind_cargo_id != 0:
                                        # print(drone)
                                        car_sum[carId] = 1
                                    break
            
            # 小车的状态为READY
            else:
                # 如果这辆小车位于初始位置 则赋值为1
                if self.des_pos_reached(self.init_pos[car_sn], car_pos, 1.0):
                    car_sum[carId] = 1
                
                    
        if sum(car_sum) == 6:
            self.state = WorkState.RUNNING_CAR
        else:
            self.state = WorkState.STOP_CAR
        # 每辆车都处于可运行状态
        
        if self.state == WorkState.RUNNING_CAR:
            # 优先检查身上有飞机的且有货物的，然后起飞
            idx = 0
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

                # Plan A
                # 远处的车不动
                # if carId == 2 or carId == 5:
                #     continue
                # Plan B
                if carId == 5:
                    # 6号小车不动
                    continue

                if self.car_state[carId] == 0:
                    # 去上货点，同时接货 
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 5.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    break

                elif self.car_state[carId] == 1 and car_drone_sn != '':
                    # 起飞无人机 缩短为1秒
                    self.fly_one_route(car_drone_sn, self.car_drone_route[car_sn], 10, 3.0, None)
                    self.car_state[carId] = 2
                    self.state == WorkState.STOP_CAR
                    break

                
                elif self.car_state[carId] == 2 and car_drone_sn != '':
                    # 返回上货点
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 5.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    break

        # 只允许起飞的状态
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
                
                # print("TTTTAKAO") 有可能是进程阻塞了
                for drone in self.drone_physical_status:
                    if car_drone_sn == drone.sn:
                        if drone.bind_cargo_id != 0:
                            print("PRE TAKEOFF")
                            self.fly_one_route(car_drone_sn, self.car_drone_route[car_sn], 10, 4.5, None)
                            self.car_state[carId] = 2
                            break
                self.state == WorkState.STOP_CAR
                break




    # 先完成静态的
    def inspect_car(self, cars):
        # 1 --> 6
        # 需要增加一个条件，其他小车不能动且状态为1
        car_sum = [0, 0, 0, 0, 0, 0]
        for car in cars:
            car_sn = car.sn
            carId = int(car_sn[-1]) - 1
            car_work_state = car.car_work_state
            car_pos = car.pos.position
            if car_work_state != 1:
                self.state = WorkState.STOP_CAR
                # break
            # 【决赛】所有小车位于起始位置
            if car_work_state == 1:
                if self.des_pos_reached(self.init_pos[car_sn], car_pos, 1.0):
                    car_sum[carId] = 1
                # 【决赛修改】
                elif self.des_pos_reached(self.work_init_pos[car_sn], car_pos, 1.0):
                    car_sum[carId] = 1

            else:
                car_sum[carId] = 0

            # ----------------动态小车调度代码----------------------- #
            # 小车在非准备状态下，动态运行代码
            # else:
            #     car_drone_sn = car.drone_sn # 小车上的飞机sn码
            #     if carId == 0 or carId == 3 : # 如果是1号小车或者4号小车
            #         if self.des_pos_reached(car_pos, self.load_pos, 0.5) == False and car_drone_sn != '':
            #             for drone in self.drone_physical_status:
            #                 # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回 # 如果飞机上有货物
            #                 if car_drone_sn == drone.sn and drone.bind_cargo_id != 0:
            #                     car_sum[carId] = 1
            #                     break
                
            #     elif carId == 1: # 2号小车
            #         if self.des_pos_reached(car_pos, self.load_pos, 0.5) == False and car_drone_sn != '':
            #             for drone in self.drone_physical_status:
            #                 # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回
            #                 if car_drone_sn == drone.sn and drone.bind_cargo_id != 0:
            #                     # 如果飞机上有货物
            #                     car_sum[carId] = 1
            #                     break

                # elif carId == 2: # 3号小车或者 5号小车
                #     if self.des_pos_reached(car_pos, self.load_pos, 11.5) == False and car_drone_sn != '':
                #         for drone in self.drone_physical_status:
                #             # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回
                #             if car_drone_sn == drone.sn and drone.bind_cargo_id != 0:
                #                 # 如果飞机上有货物
                #                 car_sum[carId] = 1
                #                 break

                # elif carId == 4: # 3号小车或者 5号小车
                #     if self.des_pos_reached(car_pos, self.load_pos, 7) == False:
                #         if car_drone_sn != '':
                #             for drone in self.drone_physical_status:
                #                 # 找到小车上的飞机编号,并判断这辆小车准备去上货还是 上货返回
                #                 if car_drone_sn == drone.sn and drone.bind_cargo_id != 0:
                #                     car_sum[carId] = 1
                #                     break
            # ----------------动态小车调度代码----------------------- #


        if sum(car_sum) == 6:
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
                car_pos = car.pos.position
                car_drone_sn = car.drone_sn
                carId = int(car_sn[-1]) - 1
                if carId == 5:
                    # 6号小车跳过
                    continue

                # 初始状态的小车
                if self.car_state[carId] == 0:
                    # 去上货点，同时接货 
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 5.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    break

                elif self.car_state[carId] == 1 and car_drone_sn != '':
                    # 起飞无人机 缩短为1秒
                    self.fly_one_route(car_drone_sn, self.car_drone_route[car_sn], 10, 3.0, None)
                    # 对于3车和5车而言，飞机起飞后需要挪动小车
                    if carId == 2:
                        self.move_car_with_route("SIM-MAGV-0003", self.three_fly_car_route, 2.0)
                    elif carId == 4:
                        self.move_car_with_route("SIM-MAGV-0005", self.five_fly_car_route, 2.0)
                    self.car_state[carId] = 2
                    self.state == WorkState.STOP_CAR
                    break

                
                elif self.car_state[carId] == 2 and car_drone_sn != '':
                    # 返回上货点
                    self.move_car_with_route(car_sn, self.car_route[car_sn], 5.0)
                    self.car_state[carId] = 1
                    self.state == WorkState.STOP_CAR
                    break


    # 对小车的运行顺序进行排序
    def sort_car_msg(self, cars):
        # 直接使用填充的方式进行排序
        process_cars = [None, None, None, None, None, None]
        # 【决赛】小车排序 [4, 6, 3, 5, 2, 1]
        for car in cars:
            car_sn = car.sn
            carId = int(car_sn[-1])
            if carId == 4 : # 1号小车
                process_cars[0] = car
            elif carId == 6: #
                process_cars[1] = car
            elif carId == 3:
                process_cars[2] = car
            elif carId == 5:
                process_cars[3] = car
            elif carId == 2:
                process_cars[4] = car
            elif carId == 1:
                process_cars[5] = car
        return process_cars
            



    def main(self):
        rospy.sleep(10.0)
        self.sys_init()
        self.state = WorkState.RUNNING_CAR
        # 4号小车可以先动
        self.move_car_with_route(car_sn='SIM-MAGV-0004', route=self.four_car_init_route, time_est=2.0)
        # 1号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0001', route=self.one_car_init_route, time_est=2.0)
        # 2号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0002', route=self.two_car_init_route, time_est=1.0)
        # 3号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0003', route=self.three_car_init_route, time_est=1.0)
        # 5号小车动
        self.move_car_with_route(car_sn='SIM-MAGV-0005', route=self.five_car_init_route, time_est=6)

        while not rospy.is_shutdown():
            # 需要增加一个车辆优先级 [1, 4], [2, 5], [3, 6]
            # 设置优先顺序
            car_msgs = self.sort_car_msg(self.car_physical_status)
            self.inspect_car(car_msgs)
            rospy.sleep(0.02)



if __name__ == "__main__":
    rospy.loginfo("--CarNode is running--")
    car = CarNode()
    car.main()
