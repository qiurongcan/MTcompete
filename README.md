# 【决赛】美团比赛运行方案

修改于10.26

最终结果：自己电脑测试2100分（20min），线上测试平台是 2300分左右波动

在自己电脑上测试**7188分**（一个小时）

## 0关键点

解题关键点：**只有一个上货操作点，因此这个节点只能是串行节点，因此需要尽量满足上货点的高效率运用**

将每一部分的动作独立，分为四个节点：

```shell
|--demo.py				以下四个节点均继承于demo中的类
|---- carNode.py 		小车节点
|---- userNode.py		用户节点
|---- droneNode.py 		飞机节点
|---- pos_move.py		额外补充的小车调度节点
```

运行流程

```shell
# 将所有代码拷贝至/home/目录下
# 在run.sh 脚本中写入
python3 /home/carNode.py &
python3 /home/userNode.py &
python3 /home/droneNode.py &
python3 /home/pos_move.py
# 最后执行 bash /home/run.sh 即可运行程序
```



## 1比赛限制

1. 飞机之间碰撞范围 5 [m]
2. 小车之间碰撞的范围 3 [m]
3. 小车与障碍之间的碰撞计算规则为质点碰撞，可按0.5 [m]计算，保证安全
4. 飞机上货需要10 [s], 飞机更换电池需要 10 [s]
5. 订单存在 **[出现时间，最佳配送时间，订单超时]** 三个时间点
6. 订单上货时间需要在 **[出现时间，订单超时]** 这个时间段上货，其他时间段无法上货
7. 在betterTime之前送达得满分，在betterTime和timeout之间送达为100～0分，线性递减，timeout之后送达为负分
8. 小车只能在操作区域和飞机起降区域运动，不能超出边界，边界判定范围可按0.5 [m]判断
9. 飞机只能在起降区域起飞或者降落，不能在用户区域执行

## 2用户节点

1. 需要放置飞机 大于2 [s]
2. 需要放置货物 10 [s]
3. 需要更换飞机
4. 需要换电池 10s
5. 需要判断到上点的小车适合送哪一个订单

只需要监测这个点
**需要的判断**：
1. 上货点附近是否有小车且小车的状态为1
2. 如果小车上没有飞机则放置飞机
3. 如果小车上有飞机且飞机的状态为1：
4. 如果电量低于30% 更换飞机或者充电，**目前采用是更换充电**，大概1小时会话费三分钟充电（6x10sx三次）
5. 飞机状态良好且电量充足，则上货
6. **选择货物时，需要满足：当前时间在orderTime之后，当前时间要在timeout之前, 最好在timeout之前300秒，避免送到是负分。需要对订单进行筛选，评定优先级**。
  筛选方法：$\delta Time = currentTime-orderTime$ current_time 大于 ordertime且越接近orderTime，同时，currentTime最好在timeout之前200秒。对筛选出来的订单进行排序评定优先级。current_time离ordertime越近，则选择这个订单**
7. 筛选得到满足的订单，则上货
8. 需要看一下哪些索引的订单优先出现，因为小车会出现没有订单可以接单的情况

**代码设定**
1. 将500个订单进行分类，目的地相同的订单分为一类，使用目的地坐标作为key值，订单使用列表形式按顺序存储并作为value值
```python
{
  des1: [cargo1, cargo7, ...],
  des2: [cargo2, cargo8, ...],
  ... ,
  des6: [cargo6, cargo12, ..],
}
```
2. 将des与car一一绑定，因此小车来的顺序需要进行合理安排
3. 小车上货完成离开上货点的时候，通过topic发布一个消息，`/run_flag` ， 表示上货点空闲，其他小车可以来上货点


## 3小车节点
1. 从接机点出发到上货点
2. 小车需要避开其他小车（3m），避免碰撞
3. 小车上货后离开上货点，前往接机点起飞
4. 小车检测是否有飞机回来
5. 这里是否需要加上到达目的地是顺手起飞？

小车初始状态，全体小车移动到一个安全位置，留出一条安全的行驶路线
生成一个小车队列，【小车之间有优先级】
六个小车位置与六个送货点匹配，生成唯一的路线
**小车需要判断**
每个小车负责**固定编号的飞机**，同时负责**固定的送货点**
小车的四种状态：
1. 航线上没有飞机，自身没有飞机，前往上货店放置飞机和货物，同时保证上货点没有飞机，同时其他五辆小车的状态为1
2. 小车上有飞机且小车位于接机点，然后其他小车状态为1，上货点没有小车，则前往上货点放置货物，如果没有，则执行3
3. 已经放飞了无人机，则持续等待无人机返回。如果返回则执行2。
4. 小车上货后，前往接机点，到达后，马上放飞无人机，放飞无人机状态变为3

## 4小车调度补充节点



## 5飞机节点

1. 监测飞机状态和飞机的位置
2. 飞机到达目的地后释放货物，并返航

**飞机需要判断**
固定飞机编号与固定目的地绑定，不接受调度
1. 检测各个飞机的位置与各个飞机与目的地的距离
2. 检测各个目的地是否有飞机和货物，如果有货物且飞机的距离目的地很近，且飞机状态为1，则释放货物，并同时遣返无人机，根据固定的飞机编号安排飞机的航线


## 需要补充的内容
路径规划：
1. 小车的路径规划，小车规划几个点，同时需要设置yaw
2. 飞机的路径规划，固定。根据不同层高的路径情况，选择速度相对较快的路径
3. 绑定：小车与飞机编号绑定，小车与接机点绑定，小车与飞机规划的路线绑定


# 存在的问题

1. 存在空单 已解决
2. 只利用了四辆小车 已解决
3. 小车的运行路线仍然需要修改 已解决
4. 小车在离开操作区域的时候，就可以起飞了 已解决
5. 动态运行小车


# 解决方法
1. 利用五辆小车
2. 不给飞机充电，直行更换一架新的飞机
3. 设计五辆小车的运动路线
4. 离开操作区域马上起飞的方法
5. 小车动态运行方法（较为简易，之前已经实现过了）


# 10.23存在问题
1. 无法实现动态小车运行
2. 需要更高效的动态调度方法

## 解决方法
多车动态运行
是否将4和5号小车拆解出来


# 10.25存在问题
1. 还不能使用6辆小车运送
2. 初始化6辆小车的位置
3. 小车布局和调度效率可以再次提升


# 10.26 BUG sim3 和sim6飞机会碰撞，需要重新规划一下航线
