# 增加一个回收飞机的节点

import rospy
from demo import DemoPipeline, WorkState
from user_pkg.msg import Position
from std_msgs.msg import Int32

class RecycleNode(DemoPipeline):
    def __init__(self, node_name='qrcRecycleNode', show=False):
        super().__init__(node_name, show)


        self.recycle_flag = -1
        self.sub_flag = rospy.Subscriber("/recycle_flag", Int32, self.flag_cb)

        self.lock_pub = rospy.Publisher('/lock', Int32, queue_size=10)

        self.init_pos = {
            "SIM-MAGV-0001": Position(187, 431, -16),  # 修改
            "SIM-MAGV-0002": Position(184, 425, -16),  # 修改
            "SIM-MAGV-0003": Position(184, 425, -16),  # 修改
            # "SIM-MAGV-0004": Position(193, 431, -16),  # 修改
            "SIM-MAGV-0005": Position(196, 424, -16),  # 修改
            "SIM-MAGV-0006": Position(196, 424, -16),  # 修改
        }


        self.recycle_route = { 
            "SIM-MAGV-0001": [Position(187, 431, -16), Position(187, 426, -16), Position(190, 425, -16)],
            "SIM-MAGV-0002": [Position(184, 425, -16), Position(190, 425, -16)],  
            "SIM-MAGV-0003": [Position(184, 425, -16), Position(190, 425, -16)],  
            # "SIM-MAGV-0004": [Position(193, 431, -16), Position(190, 425, -16)],  
            "SIM-MAGV-0005": [Position(196, 424, -16), Position(190, 425, -16)],  
            "SIM-MAGV-0006": [Position(196, 424, -16), Position(190, 425, -16)],  
        }


    def flag_cb(self, msg):
        self.recycle_flag = msg.data

    # 回收飞机
    def recycle_drones(self):
        while self.car_physical_status is None:
            pass


        for car in self.car_physical_status:
            car_sn = car.sn
            car_work_state = car.car_work_state
            car_drone_sn = car.drone_sn
            car_pos = car.pos.position
            carId = int(car_sn[-1]) - 1
            if carId == 3:
                continue

            if car_work_state == 1 and car_drone_sn != '' and self.des_pos_reached(car_pos, self.init_pos[car_sn], 1.2) and self.recycle_flag == 1:
                # 解决2和3车之间的博弈关系、 5和6车之间的博弈关系
                # TODO 需要有一个优先级的顺序
                if carId == 0:
                    self.move_car_with_route(car_sn, self.recycle_route[car_sn], 2.0)
                    self.recycle_flag = 0

                if carId == 1:
                    self.move_car_with_route(car_sn, self.recycle_route[car_sn], 2.0)
                    self.recycle_flag = 0
                
                elif carId == 2:
                    self.move_car_with_route(car_sn, self.recycle_route[car_sn], 2.0)
                    self.recycle_flag = 0
                
                # elif carId == 3: # 4号小车
                #     self.move_car_with_route(car_sn, self.recycle_route[car_sn], 2.0)
                #     self.recycle_flag = 0
                
                elif carId == 4:
                    self.move_car_with_route(car_sn, self.recycle_route[car_sn], 2.0)
                    self.recycle_flag = 0
                
                elif carId == 5:
                    self.move_car_with_route(car_sn, self.recycle_route[car_sn], 2.0)
                    self.recycle_flag = 0
                
                # 对四号小车上锁
                self.lock_pub.publish(0)
        
    
    def main(self):
        
        while not rospy.is_shutdown():
            self.recycle_drones()





if __name__ == "__main__":

    recycleNode = RecycleNode()
    recycleNode.main()





