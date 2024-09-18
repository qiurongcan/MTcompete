#!/usr/bin/env python3
import rospy
import json
import numpy as np
from enum import Enum
# 导入说有消息
from std_msgs.msg import String
from user_pkg.msg import BillStatus
from user_pkg.msg import BindCargo
from user_pkg.msg import CarPhysicalStatus
from user_pkg.msg import CarRouteInfo
from user_pkg.msg import DroneMsg
from user_pkg.msg import DronePhysicalStatus
from user_pkg.msg import DroneWayPoint
from user_pkg.msg import DroneWayPointInfo
from user_pkg.msg import DynamicPosition
from user_pkg.msg import EulerAngle
from user_pkg.msg import EventMsg
from user_pkg.msg import PanoramicInfo
from user_pkg.msg import Position
from user_pkg.msg import UnBindInfo
from user_pkg.msg import UserCmdRequest
from user_pkg.msg import UserCmdResponse
from user_pkg.msg import UserPhysicalStatus
from user_pkg.msg import Voxel
from user_pkg.srv import QueryVoxel, QueryVoxelRequest

# demo定义的状态流转


class WorkState(Enum):
    """
    定义13中状态
    """
    START = 1                                       # 开始状态 
    TEST_MAP_QUERY = 2                              # 测试地图体素状态

    MOVE_CAR_GO_TO_LOADING_POINT = 3                # 小车1 -> 上货点
    MOVE_DRONE_ON_CAR = 4                           # 飞机X -> 小车1
    MOVE_CARGO_IN_DRONE = 5                         # 货物X -> 飞机X
    MOVE_CAR_GO_TO_PICKUP = 6                       # 小车2 -> 接机点
    MOVE_CAR_GO_TO_DROPOFF =7                       # 小车1 -> 送机点

    MOVE_CAR_TO_LEAVING_POINT = 8                   # 小车（载货物和飞机） -> 出发点（飞机可以起飞的位置）
    RELEASE_DRONE_OUT = 9                           # 飞机（载货物）离开 《此处需要有航线》
    RELEASE_CARGO = 10                              # 释放货物（到达目的地）
    RELEASE_DRONE_RETURN = 11                       # 飞机返航
    MOVE_CAR_BACK_TO_LOADING_POINT = 12             # 小车 -> 上货点
    # 11 和 12 插入到3和4 之间
    DRONE_BATTERY_REPLACEMENT = 13                  # 更换飞机电池
    DRONE_RETRIEVE = 14                             # 飞机回收
    FINISHED = 15                                   # 完成
    WAITING_DRONE = 16                              # 等待飞机回来
    
    # 用户节点的状态
    WAIT_CAR = 17

    # 小车节点
    RUNNING_CAR = 18
    STOP_CAR = 19



class DemoPipeline:
    def __init__(self, node_name='qrc_car1',show=True):
        # 初始化ros全局变量
        self.state = WorkState.START
        rospy.init_node(node_name)
        # 设置命令发布者
        self.cmd_pub = rospy.Publisher(
            '/cmd_exec', 
            UserCmdRequest, 
            queue_size=10000
            )
        # 订阅全局信息
        self.info_sub = rospy.Subscriber(
            '/panoramic_info',
            PanoramicInfo,
            self.panoramic_info_callback,
            queue_size=10
            )
        # 订阅地图信息
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)
        # 读取配置文件和信息
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        self.drone_infos = self.config['taskParam']['droneParamList']               # 读取所有飞机的信息
        self.car_infos = self.config['taskParam']['magvParamList']                  # 读取所有小车的信息
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']    # 读取上货地点
        self.map_boundary = self.config['taskParam']['mapBoundaryInfo']             # 读取地图边界信息
        self.waybill_infos = self.config['taskParam']['waybillParamList']           # 读取所有订单信息
        # 释放货物的位置只有两个
        self.unloading_cargo_stations = self.config['taskParam']['unloadingCargoStationList']
        self.drone_sn_list = [drone['droneSn'] for drone in self.drone_infos]       # 飞机的sn码 唯一标识
        self.car_sn_list = [car['magvSn'] for car in self.car_infos]                # 小车的sn码
        self.peer_id = self.config['peerId']                                        # 选手的PeerId
        self.task_guid = self.config['taskParam']['guid']                           # task_guid
        # 小车、飞机、订单、分数、事件等状态
        self.car_physical_status = None
        self.drone_physical_status = None
        self.bills_status = None
        self.score = None
        self.events = None
        self.show = show


    def inspect_all_state(self, msg):
        """
        在终端显示所有飞机的状态
        """
        cars = msg.cars
        drones = msg.drones
        bills = msg.bills
        events = msg.events
        score = msg.score
        # 字符长度为13+7+4+1空格 25

        state_info = f"""
        --------- state_inspect --------- \n
        ===============================  Score: {score} ==============================
        ==================  Process State: {self.state:<15} ==========================
        ----------------------------------  Car  ---------------------------------------
        Car_SN: {cars[0].sn:<17}|Car_SN: {cars[1].sn:<17}
        State: {cars[0].car_work_state:<18}|State: {cars[1].car_work_state:<20}|-----Car_State-----
        x: {cars[0].pos.position.x:<22}|x: {cars[1].pos.position.x:<24}|0  : Car_Unknown
        y: {cars[0].pos.position.y:<22}|y: {cars[1].pos.position.y:<24}|1  : Car_Ready
        z: {cars[0].pos.position.z:<22}|z: {cars[1].pos.position.z:<24}|2  : Car_Running
        v: {cars[0].pos.v:<22}|v: {cars[1].pos.v:<24}|10 : Car_Error
        Drone_SN: {cars[0].drone_sn:<15}|Drone_SN: {cars[1].drone_sn:<15}
        ----------------------------------  Drone  -------------------------------------
        Drone_SN: {drones[0].sn:<15}|Drone_SN: {drones[1].sn:<15}|Drone_SN: {drones[2].sn:<15}
        State: {drones[0].drone_work_state:<18}|State: {drones[1].drone_work_state:<18}|State: {drones[2].drone_work_state:<18}
        Battery: {drones[0].remaining_capacity:<15}%|Battery: {drones[1].remaining_capacity:<15}%|Battery: {drones[2].remaining_capacity:<15}%
        Cargo_Id: {drones[0].bind_cargo_id:<15}|Cargo_Id: {drones[1].bind_cargo_id:<15}|Cargo_Id: {drones[2].bind_cargo_id:<15}
        x: {drones[0].pos.position.x:<22}|x: {drones[1].pos.position.x:<22}|x: {drones[2].pos.position.x:<22}
        y: {drones[0].pos.position.y:<22}|y: {drones[1].pos.position.y:<22}|y: {drones[2].pos.position.y:<22}
        z: {drones[0].pos.position.z:<22}|z: {drones[1].pos.position.z:<22}|z: {drones[2].pos.position.z:<22}
        v: {drones[0].pos.v:<22}|v: {drones[1].pos.v:<22}|v: {drones[2].pos.v:<22}

        Drone_SN: {drones[3].sn:<15}|Drone_SN: {drones[4].sn:<15}|
        State: {drones[3].drone_work_state:<18}|State: {drones[4].drone_work_state:<18}|----------Drone_State----------
        Battery: {drones[3].remaining_capacity:<15}%|Battery: {drones[4].remaining_capacity:<15}%|0  : Unkown   1  : Ready
        Cargo_Id: {drones[3].bind_cargo_id:<15}|Cargo_Id: {drones[4].bind_cargo_id:<15}|2  : Takeoff  3  : Flying
        x: {drones[3].pos.position.x:<22}|x: {drones[4].pos.position.x:<22}|4  : Landing  10 : Error
        y: {drones[3].pos.position.y:<22}|y: {drones[4].pos.position.y:<22}|5  : Charging_Battery
        z: {drones[3].pos.position.z:<22}|z: {drones[4].pos.position.z:<22}|6  : Loading_cargo
        v: {drones[3].pos.v:<22}|v: {drones[4].pos.v:<22}|
        """
        rospy.loginfo(state_info)

    # 仿真回调函数，获取实时信息
    def panoramic_info_callback(self, panoramic_info):
        """
        编写一个实时信息界面
        """
        self.car_physical_status = panoramic_info.cars
        self.drone_physical_status = panoramic_info.drones
        self.bills_status = panoramic_info.bills
        self.score = panoramic_info.score
        self.events = panoramic_info.events
        # 在终端显示无人机和小车的状态
        if self.show:
            self.inspect_all_state(panoramic_info)


    # 系统初始化(按需)
    def sys_init(self):
        rospy.sleep(10.0)
        self.state = WorkState.TEST_MAP_QUERY

    # 测试地图查询接口，可用这个或地图SDK进行航线规划
    def test_map_query(self):
        # TODO 需要修改为更智能的查询
        request = QueryVoxelRequest()
        request.x = 1.0
        request.y = 2.0
        request.z = -3.0
        response = self.map_client(request)
        # print(response)
        if response.success:
            self.state = WorkState.MOVE_CAR_GO_TO_LOADING_POINT

    # 移动地面车辆的函数
    def move_car_with_start_and_end(
            self, car_sn, start, end, time_est, next_state, yaw=0.0):
        """
        小车需要做的：
            1.初始点 -> 上货点
            2.上货点 -> 卸货点
            3.卸货点 -> 上货点
            4.小车路径[[x1, y1, z1], [x2, y2, z2], ...]
        """
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        # 小车路线也可以是多个点
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        msg.car_route_info.yaw = yaw
        # for _ in range(10):
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

    # 往车上挪机
    def move_drone_on_car(self, car_sn, drone_sn, time_est, next_state):
        """
        设计：无人机、小车分配算法
        """
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_CAR
        msg.binding_drone.car_sn = car_sn
        msg.binding_drone.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 往飞机上挂餐
    def move_cargo_in_drone(self, cargo_id, drone_sn, time_est, next_state=WorkState.MOVE_CAR_TO_LEAVING_POINT):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        # 难道发消息的时候丢包了？
        msg.binding_cargo.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 飞机航线飞行函数
    def fly_one_route(self, drone_sn, route, speed, time_est, next_state):
        """
        1.初始点 -> 小车卸货点
        2.小车卸货点 -> 飞机卸货点
        3.飞机卸货点 -> 小车卸货点
        4.规划飞机飞行航线 TAKEOFF -> WAYLINE -> LAND
        """
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_EXEC_ROUTE
        msg.drone_way_point_info.droneSn = drone_sn
        takeoff_point = DroneWayPoint()
        takeoff_point.type = DroneWayPoint.POINT_TAKEOFF
        takeoff_point.timeoutsec =  1000
        # 第一个位置是起飞点
        msg.drone_way_point_info.way_point.append(takeoff_point)
        for waypoint in route:
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING
            middle_point.pos.x = waypoint.x
            middle_point.pos.y = waypoint.y
            middle_point.pos.z = waypoint.z
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            msg.drone_way_point_info.way_point.append(middle_point)
        land_point = DroneWayPoint()
        land_point.type = DroneWayPoint.POINT_LANDING
        land_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(land_point)
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 抛餐函数
    def release_cargo(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 回收飞机函数 这个函数不一定需要使用
    def drone_retrieve(self, drone_sn, car_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_BIRTHPLACE
        msg.unbind_info.drone_sn = drone_sn
        msg.unbind_info.car_sn = car_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state
    # 状态流转主函数

    def running(self):
        rospy.sleep(2.0)
        waybill_count = 0
        car_sn = self.car_sn_list[0]        # 只使用了一辆小车
        drone_sn = self.drone_sn_list[0]    # 只使用了一辆飞机
        # 根据小车的sn码读取这辆小车的所有状态信息
        car_physical_status = next(
            (car for car in self.car_physical_status if car.sn == car_sn), None)
        # 获取小车的起始位置
        car_init_pos = car_physical_status.pos.position
        while not rospy.is_shutdown() and waybill_count < len(self.waybill_infos):
            # 根据编号获取订单信息
            waybill = self.waybill_infos[waybill_count]
            # 获取该sn码小车的所有信息
            car_physical_status = next(
                (car for car in self.car_physical_status if car.sn == car_sn), None)
            car_pos = car_physical_status.pos.position
            # 获取该sn码飞机的所有信息
            drone_physical_status = next(
                (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
            drone_pos = drone_physical_status.pos.position
            loading_pos = Position(
                self.loading_cargo_point['x'],
                self.loading_cargo_point['y'],
                self.loading_cargo_point['z'])
            # print(self.state)
            # 初始化
            if self.state == WorkState.START:
                self.sys_init()
            elif self.state == WorkState.TEST_MAP_QUERY:
                self.test_map_query()
            # 小车到加载货物的位置 -> 下一状态为飞机到车
            elif self.state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                # 这个5s可能会有点长
                self.move_car_with_start_and_end(
                    car_sn, car_pos, loading_pos, 2.0, WorkState.MOVE_DRONE_ON_CAR)
            # 飞机搭载到小车时需要有前置条件
            elif self.state == WorkState.MOVE_DRONE_ON_CAR:
                # 将飞机移动到小车上5s 可能有点长
                if(self.des_pos_reached(loading_pos, car_pos, 0.5) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    self.move_drone_on_car(
                        car_sn, drone_sn, 0.02, WorkState.MOVE_CARGO_IN_DRONE)
            # 货物上飞机
            elif self.state == WorkState.MOVE_CARGO_IN_DRONE:
                cargo_id = waybill['cargoParam']['index']
                self.move_cargo_in_drone(cargo_id, drone_sn, 10.0)
            # TODO 代码可能有问题
            elif self.state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                self.move_car_with_start_and_end(
                    car_sn, car_pos, car_init_pos, 2.0, WorkState.RELEASE_DRONE_OUT)
            # 无人机按照航线飞行
            elif self.state == WorkState.RELEASE_DRONE_OUT:
                if(self.des_pos_reached(car_init_pos, car_pos, 0.5) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    start_pos = Position(drone_pos.x, drone_pos.y, -145)
                    middle_pos = Position(
                        waybill['targetPosition']['x'], waybill['targetPosition']['y'], -145)
                    end_pos = Position(
                        waybill['targetPosition']['x'],
                        waybill['targetPosition']['y'],
                        waybill['targetPosition']['z'] - 5)
                    route = [start_pos, middle_pos, end_pos]
                    self.fly_one_route(
                        drone_sn, route, 15.0, 10, WorkState.RELEASE_CARGO)
            # 无人机释放货物
            elif self.state == WorkState.RELEASE_CARGO:
                des_pos = Position(
                    waybill['targetPosition']['x'],
                    waybill['targetPosition']['y'],
                    waybill['targetPosition']['z'])
                if(self.des_pos_reached(des_pos, drone_pos, 2.0) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    self.release_cargo(
                        drone_sn, 5.0, WorkState.RELEASE_DRONE_RETURN)
            # 无人机返航
            elif self.state == WorkState.RELEASE_DRONE_RETURN:
                des_pos = Position(
                    waybill['targetPosition']['x'],
                    waybill['targetPosition']['y'],
                    waybill['targetPosition']['z'])
                if(self.des_pos_reached(des_pos, drone_pos, 2.0) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    start_pos = Position(drone_pos.x, drone_pos.y, -145) # 起飞以后上升的高度
                    middle_pos = Position(car_init_pos.x, car_init_pos.y, -145)
                    end_pos = Position(car_pos.x, car_pos.y, car_pos.z - 20) # 返航至小车的位置
                    route = [start_pos, middle_pos, end_pos]
                    self.fly_one_route(
                        drone_sn, route, 15.0, 10, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
            # 小车返回上货点
            elif self.state == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT:
                if(self.des_pos_reached(car_pos, drone_pos, 0.8) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    self.move_car_with_start_and_end(
                        car_sn, car_pos, loading_pos, 5.0, WorkState.DRONE_BATTERY_REPLACEMENT)
            # 更换无人机电池
            elif self.state == WorkState.DRONE_BATTERY_REPLACEMENT:
                if(self.des_pos_reached(loading_pos, car_pos, 0.8) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    self.battery_replacement(
                        drone_sn, 10.0, WorkState.DRONE_RETRIEVE)
            # 无人机完成任务，订单+1
            elif self.state == WorkState.DRONE_RETRIEVE:
                self.drone_retrieve(
                    drone_sn, car_sn, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                waybill_count += 1
            rospy.sleep(1.0)
        print(
            'Total waybill finished: ',
            waybill_count,
            ', Total score: ',
            self.score)
        
    def main(self):
        """在此处编写自定义算法"""
        raise NotImplementedError("Please Code Here")


if __name__ == '__main__':
    # running Demo
    main = DemoPipeline()
    main.running()




"""
1.在训练集中 需要送货的地点只有两个 dest1 = [530, 388, -20] dest2 = [590, 392, -16]
2.小车在 (car_init_pos, load_cargo_pos) 之间往返 car_init_pos可以时航空作业区的任何点
3.飞机在 ()
4.最多飞行1000s
5.飞机上货、换电时间为10s
6.飞机最大飞行速度为10m/s
7.需要在距离起飞点(x, y)半径10米范围内完成  [60, 120]
8.降落距离地面至少3米
9.无人机撞击距离为5米, 小车撞击距离为3米
10.挂餐可以不按照顺序
11.无人机发送航线时, 空中无法更改航线
实现方案：
1.根据目的地 寻找往返的航线
2.求解小车往返、 上下卸货&换电、
3.设计两个节点 一个节点控制一辆小车 一个小车负责一个目的地的航线
4.设置两个地点 一个负责送无人机 一个负责接无人机
5.需要两个节点
    CarNode 负责小车1和2的运行管理
    DroneNode 负责所有飞机的运行管理

约束条件1
    假设现在有两架飞机1和2 飞往同一个目的地 1先起飞 2随后起飞 1和2飞同一条航线 
    1和2之间的时间间隔需要满足 1到达目的地后-降落-卸货-起飞返航离开目的地xy 2还没有到目的地xy
    1 返回时 如果存在3(4) 则需要在3(4)起飞离开xy后 才能到达

实现流程
    初始化
    1 标记小车1 和 小车2 的初始位置
    小车1初始位置: 接机点      小车2初始位置: 送机点
    执行流程
    小车1 -> 上货点 -> 上货(10s) 小车2在小车1到达上货点时: 小车2 -> 接机点
    小车1 -> 送机点     小车2在小车1到达送机点时: 小车2 -> 上货点 -> 上货(10s)
    小车1 -> 接机点     小车2在小车1到达接机点且自身已经上货完成后 小车2 -> 送机点
    小车1 -> 上货点 -> 上货(10s) 同样小车2在小车1到达上货点时 小车2 -> 接机点

    注意
    某一辆小车在接机点时 需要停留一段时间 确保飞机回来时可以接到 !停留时间如何设置
    接机点被占用时 另一辆小车可以做的操作有 在 送机点和 上货点往返 前提是出生点存在飞机且航线未饱和的情况

    首先是规划三个点之间的最优路径 貌似时平底 两点之间直线距离最短
    需要增加一个rostopic 用于 CarNode 和 DroneNode 之间的状态通信
    小车在运行过程中 到达目标位置时 会出现累计误差 现在有一个问题就是如何消除累计误差 小车能够一直运行
    相同目的地一直送 送完为止 进行下一个目的地

"""

