# use to inspect the global status
# including score, drones, cars, users, bills,
# Date: 10.19 

import rospy
import numpy as np
from demo import DemoPipeline, WorkState
import time


class Inspect(DemoPipeline):
    def __init__(self, node_name='qrcInspect', show=False):
        super().__init__(node_name, show)

    
    def inspect_bills(self):
        # 保证获取的信息不为None
        if self.bills_status is not None:
            for bill in self.bills_status:
                # 当前时间的单位 [s]
                current_time = time.time()
                # print("current_time:", current_time)
                # 订单中给出的时间戳单位 [ms]
                ordertime = bill.orderTime
                bettertime = bill.betterTime
                timeout = bill.timeout
                # 1.需要满足 current_time > ordertime
                delta_time = current_time - ordertime/1000


                # 2.需要满足current_time < timeout
                is_timeout = current_time - timeout/1000
                if is_timeout >= 0:
                    # 这个订单无法配送
                    print("无法配送这个订单")
                else:
                    # 这个订单可以配送，最好在bettertime之前
                    print("可以配送这个订单")
                # print("ordertime:", ordertime)

                # compare current and ordertime
                
                # print("delta time is ", delta_time)
                # 这个间隔的时间在 [120, 150] 单位[s] 之间
                print("time1:", bettertime-ordertime)
                # 这个间隔固定为 300 [s]
                print("time2:", timeout-bettertime)


                break 


    def main(self):
        while True:
            self.inspect_bills()


if __name__ == "__main__":
    inspect = Inspect()
    inspect.main()
