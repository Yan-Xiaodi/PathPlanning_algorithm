# PathPlanning_algorithm

## 更新

- **2022.6.23**: 新增了GRRT-Connect算法，这个算法由本项目提出，基于RRT-Connect算法改进，具有更好的效果

项目实现了在2D Map下的RRT、RRT*、RRT-Connect的路径规划算法，支持自定义障碍物，异步保存轨迹到视频流

## 搭建

### 项目依赖

- OpenCV(>=3.0)

```shell
git clone https://github.com/Yan-Xiaodi/PathPlanning_algorithm.git
mkdir build
cd build
cmake ..
make 
./path_planning
```

## 自定义障碍物

通过`Map::drawObstacles()`启动绘制流程

- 鼠标点击生成一个顶点，必须按照顺时针方向逐个绘制顶点(数量不限)
- 绘制完一个障碍物后的所有顶点后，按下空格即可生成障碍物
- 重复1-2步，绘制其他障碍物
- 所有障碍物绘制完毕后，按下空格，退出绘制流程

如下所示

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/draw.gif?raw=true" alt="draw.gif" width="70%" />

## 使用

参数设置

```C++
const float stride = 10;        //扩展树的步长
const float threashold = 0.1;   //分界概率值
const int MaxIteration = 20000; //迭代上限
const int rand_precition = 5;   //坐标精确位数
```

地图设置

```C++
cv::Mat planMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
//创建地图
Map map(width, height, planMap, window_name3);
//地图初始化
map.init();
//绘制障碍物
map.drawObstacles();
```

选择相应的算法进行planning()

```C++
RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition,
                        map,radius);
rrt_star.init();
rrt_star.planning();
```

## 结果示例

参数

```C++
stride=10 threashold=0.1 MaxIteration=20000 rand_precition=5  (rrt*:radius=40)
```

### RRT

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/RRT.gif?raw=true" alt="RRT.gif" width="60%" />

### RRT*

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/RRT_star.gif?raw=true" alt="RRT_star.gif" width="60%" />

### RRT-Connect

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/RRT_connect.gif?raw=true" alt="RRT_connect.gif" width="60%" />

### 比较

|  Algorithm  | 迭代次数 |   时间(s)   |  路径代价   |
| :---------: | :------: | :------: | :---------: |
|     RRT     |   1645   |   7.87   |   1892.06   |
|    RRT*     |   2309   |   9.91   |   1652.11   |
| RRT-Connect | **195**  | **0.60** | **1646.95** |

## TODO

- PRM Series algorithm
- RRT-3D
- A*、 Dijkstra
- ....

