# PathPlanning_algorithm

### [中文说明](.\README-CN.md)

This Project realize some path-planning algorithm such as RRT 、RRT*.Each algorithm has been designed as class.

## Build Project

### depencies

- OpenCV(>=3.0)

```shell
git clone https://github.com/Yan-Xiaodi/PathPlanning_algorithm.git
mkdir build
cd build
cmake ..
make 
./path_planning
```

## Create Obstacle throw mouse

Through `Map::drawObstacles()` you can use mouse to create the obstacle(must be Convex) in the Screen

- when you clicked Mouse down ,you create a vertex of obstacle
- you must follow clockwise order to create vertex
- if you finished to  draw a obstacle ,just press the space,it will record the obstacle infomation ,and you can draw your next obstacle
- when you have drawn all the obstacles,also press the space (follow the pre-press-space ),the map will be finished.

See examples

![draw.gif](https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/draw.gif?raw=true)

## use the algorithm

Set the parameter of algorithm

```C++
const float stride = 10;        //扩展树的步长
const float threashold = 0.1;   //分界概率值
const int MaxIteration = 20000; //迭代上限
const int rand_precition = 5;   //坐标精确位数
```

Set the map

```C++
cv::Mat planMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
//创建地图
Map map(width, height, planMap, window_name3);
//地图初始化
map.init();
//绘制障碍物
map.drawObstacles();
```

choose one alogrithm to planning

```c++
RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition,
                        map,radius);
rrt_star.init();
rrt_star.planning();
```

## Examples

**parameter:**

```C++
stride=10 threashold=0.1 MaxIteration=20000 rand_precition=5  (rrt*:radius=40)
```



### RRT

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/RRT.gif?raw=true" alt="RRT.gif" style="zoom:80%;" />

### RRT*

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/RRT_star.gif?raw=true" alt="RRT_star.gif" style="zoom:80%;" />

### RRT-Connect

<img src="https://github.com/Yan-Xiaodi/Image_respority/blob/master/img/RRT_connect.gif?raw=true" alt="RRT_connect.gif" style="zoom:80%;" />

### Comparision

|  Algorithm  | Iteration |   time   |  path-cost  |
| :---------: | :-------: | :------: | :---------: |
|     RRT     |   1645    |   7.87   |   1892.06   |
|    RRT*     |   2309    |   9.91   |   1652.11   |
| RRT-Connect |  **195**  | **0.60** | **1646.95** |

## TODO

- PRM Series algorithm
- RRT-3D
- A*、 Dijkstra
- ....

