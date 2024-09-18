# 美团比赛运行方案
**关键点：只有一个上货操作点，因此这个节点只能是串行节点**
 一共运行三个节点
 首先要明确每一个节点需要做什么

## 用户节点
1. 需要放置飞机 大于2s
2. 需要放置货物 10s
3. 需要更换飞机
4. 需要换电池 10s

只需要监测这个点
**需要的判断**：
1. 上货点附近是否有小车且小车的状态为1
2. 如果小车上没有飞机则放置飞机
3. 如果小车上有飞机且飞机的状态为1：
4. 如果电量低于30% 更换飞机或者充电
5. 飞机状态良好且电量充足
6. 则上货

## 小车节点
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

## 飞机节点
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

## 测试的效果
1. v3是基础版本 best score is 12 20min
2. v5是改进后的版本 best score is 15 20min； 1 hour is 45 score 这个版本是最保守的版本
3. 增加一个激进的版本
4. v6版本是调整了以下小车的送货顺序 score:15 in 20 min Stable Version

## TODO
1. 增加一个调整 与小车匹配送货点的版本；同时调整小车到上货点的优先级 (已完成) 需要在car、user、drone三个节点都做调整
