#include "Grrt_connect.h"
#include "rrt.h"
#include "rrt_connect.h"
#include "rrt_star.h"

using namespace std;

Obstacle createRectangle(const int width, const int height, cv::Point2i center)
{
    Obstacle obj;
    obj.push(center.x - width / 2, center.y - height / 2);
    obj.push(center.x + width / 2, center.y - height / 2);
    obj.push(center.x + width / 2, center.y + height / 2);
    obj.push(center.x - width / 2, center.y + height / 2);
    return obj;
}

bool checkCollision(cv::Point2f& point, Obstacle& obj)
{
    std::function<bool(cv::Point2f&, cv::Point2f&, cv::Point2f&)> isInside =
        [&](cv::Point2f& a, cv::Point2f& b, cv::Point2f& c) {
            return ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) >= 0;
        };

    //如果点在多边形内部(包含边缘)，则势必满足点与所有边构成的叉乘向量大于等于0.这种情况是发生碰撞
    //只要有一例不符，则未发生碰撞
    bool collision = true;
    for (int i = 0; i < obj.getPointsSize(); ++i) {
        int j = (i == obj.getPointsSize() - 1) ? 0 : i + 1;
        if (!isInside(obj.getVertices()[i], obj.getVertices()[j], point)) {
            //只要有一个点不是inside，说明没有和这个物体发生碰撞
            collision = false;
            break;
        }
    }
    return collision;
}

cv::Point2i createCenterRandom(const int width, const int height, int win_width, int win_height)
{
    int maxL = max(width, height);
    win_width -= maxL;
    win_height -= maxL;
    float seed = rand() % (int)pow(10, 3) / (float)pow(10, 3);
    printf("%f\n", seed);
    cv::Point2i center;
    center.x = static_cast<int>(seed * win_width);
    center.y = static_cast<int>(seed * win_height);
    return center;
}

void addObstaclesRandom(Map& map, cv::Point2f& start, cv::Point2f& end)
{
    srand(time(NULL));
    int rectangle_w1 = 100, rectangle_h1 = 100;
    int rectangle_w2 = 50, rectangle_h2 = 200;
    int rectangle_w3 = 100, rectangle_h3 = 120;
    int nums = 0;
    while (nums < 3) {
        cv::Point2i center1 =
            createCenterRandom(rectangle_w1, rectangle_h1, map.getWidth(), map.getHeight());
        cv::Point2i center2 =
            createCenterRandom(rectangle_w2, rectangle_h2, map.getWidth(), map.getHeight());
        cv::Point2i center3 =
            createCenterRandom(rectangle_w3, rectangle_h3, map.getWidth(), map.getHeight());
        Obstacle obj1 = createRectangle(rectangle_w1, rectangle_h1, center1);
        Obstacle obj2 = createRectangle(rectangle_w2, rectangle_h2, center2);
        Obstacle obj3 = createRectangle(rectangle_w3, rectangle_h3, center3);
        if (!checkCollision(start, obj1) && !checkCollision(end, obj1)) {
            map.addObstacle(obj1);
            nums++;
        }
        if (!checkCollision(start, obj2) && !checkCollision(end, obj2)) {
            map.addObstacle(obj2);
            nums++;
        }
        if (!checkCollision(start, obj3) && !checkCollision(end, obj3)) {
            map.addObstacle(obj3);
            nums++;
        }
    }
}

Map setMapWithRandomObj(const std::string& window_name, const int width, const int height,
                        cv::Point2f& start, cv::Point2f& end)
{
    cv::Mat rrtImage(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    Map rrtMap(width, height, rrtImage, window_name);
    rrtMap.init();
    addObstaclesRandom(rrtMap, start, end);
    rrtMap.drawAllObstacles();
    return rrtMap;
}

void solve(const std::string& plannerName, Map* mapPtr, const float& stride,
           const float& threashold, const int& MaxIteration, const int& rand_precition,
           const float& radius, const float& grow_rate, cv::Point2f& start, cv::Point2f& end)
{
    RRT* planner;
    if (plannerName == "rrt") {
        RRT rrt(start, end, stride, threashold, MaxIteration, rand_precition, mapPtr);
        rrt.init();
        rrt.planning();
        planner = &rrt;
    } else if (plannerName == "rrt_star") {
        RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition, mapPtr,
                          radius);
        rrt_star.init();
        rrt_star.planning();
        planner = &rrt_star;
    } else if (plannerName == "rrt_connect") {
        RRT_Connect rrt_connect(start, end, stride, threashold, MaxIteration, rand_precition,
                                mapPtr);
        rrt_connect.init();
        rrt_connect.planning();
        planner = &rrt_connect;
    } else if (plannerName == "Grrt_connect") {
        GRRT_Connect Grrt_connect(start, end, stride, threashold, MaxIteration, rand_precition,
                                  mapPtr, grow_rate);
        Grrt_connect.init();
        Grrt_connect.planning();
        planner = &Grrt_connect;
    }
    const string iteration3 = "itreation: " + to_string(planner->getIteration());
    const string time3 = "time cost: " + to_string(planner->getPlanTime());
    const string length3 = "plan length: " + to_string(planner->getPlanLength());
    mapPtr->drawText(iteration3, cv::Point(20, 920));
    mapPtr->drawText(time3, cv::Point(20, 950));
    mapPtr->drawText(length3, cv::Point(20, 980));
}

void compare_planner(const string& win1, const string& win2, const string& win3, const string& win4,
                     const int& width, const int& height, const float& stride,
                     const float& threashold, const int& MaxIteration, const int& rand_precition,
                     const float& radius, const float grow_rate, cv::Point2f& start,
                     cv::Point2f& end)
{
    Map map1 = setMapWithRandomObj(win1, width, height, start, end);
    Map map2 = setMapWithRandomObj(win2, width, height, start, end);
    Map map3 = setMapWithRandomObj(win3, width, height, start, end);
    Map map4 = setMapWithRandomObj(win4, width, height, start, end);

    solve("rrt", &map1, stride, threashold, MaxIteration, rand_precition, radius, grow_rate, start,
          end);
    solve("rrt_star", &map2, stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
          start, end);
    solve("rrt_connect", &map3, stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
          start, end);
    solve("Grrt_connect", &map4, stride, threashold, MaxIteration, rand_precition, radius,
          grow_rate, start, end);

    cv::imshow(win1, map1.getPlanMap());
    cv::imshow(win2, map2.getPlanMap());
    cv::imshow(win3, map3.getPlanMap());
    cv::imshow(win4, map4.getPlanMap());
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    const string window_name1 = "RRT Path Planning";
    const string window_name2 = "RRT* Path Planning";
    const string window_name3 = "RRT-Connect Path Planning";
    const string window_name4 = "GRRT-Connect Path Planning";
    //设置地图的宽和高
    const int width = 1000;
    const int height = 1000;
    const float stride = 10;        //扩展树的步长
    const float threashold = 0.1;   //分界概率值
    const int MaxIteration = 20000; //迭代上限
    const int rand_precition = 5;   //坐标精确位数
    cv::Point2f start(0, 0), end(900, 900);
    float radius = 40;
    float grow_rate = 1.01;

    // compare_planner(window_name1, window_name2, window_name3, window_name4, width, height,
    // stride,
    //                 threashold, MaxIteration, rand_precition, radius, grow_rate, start, end);
    cv::Mat planMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //创建地图
    Map map(width, height, planMap, window_name4);
    //地图初始化
    map.init();

    //绘制障碍物
    map.drawObstacles();
    // RRT算法
    // RRT rrt(start, end, stride, threashold, MaxIteration, rand_precition, &map);
    // rrt.init();
    // rrt.planning();

    // RRT*算法
    // RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition,
    // &map,
    //                   radius);
    // rrt_star.init();
    // rrt_star.planning();

    // RRT_Connect算法
    // RRT_Connect rrt_connect(start, end, stride, threashold, MaxIteration, rand_precition, &map);
    // rrt_connect.init();
    // rrt_connect.planning();

    // GRRT_Connect算法
    GRRT_Connect Grrt_connect(start, end, stride, threashold, MaxIteration, rand_precition, &map,
                              grow_rate);
    Grrt_connect.init();
    Grrt_connect.planning();
    cv::waitKey(0);
}
