#include "map.h"
#include <sys/time.h>
using namespace std;

bool checkTwoLineCollision(cv::Point2f p1, cv::Point2f p2, cv::Point2f q1, cv::Point2f q2)
{
    //判断线段是否相交的方法:线段叉乘法

    cv::Point2f a1(p2.x - p1.x, p2.y - p1.y);
    cv::Point2f b1(q1.x - p1.x, q1.y - p1.y);
    cv::Point2f c1(q2.x - p1.x, q2.y - p1.y);

    cv::Point2f a2(q2.x - q1.x, q2.y - q1.y);
    cv::Point2f b2(p1.x - q1.x, p1.y - q1.y);
    cv::Point2f c2(p2.x - q1.x, p2.y - q1.y);

    float condition1 = (a1.x * b1.y - a1.y * b1.x) * (a1.x * c1.y - a1.y * c1.x);
    float condition2 = (a2.x * b2.y - a2.y * b2.x) * (a2.x * c2.y - a2.y * c2.x);
    return condition1 <= 0 && condition2 <= 0;
}

bool checkCollisionPoint(cv::Point2f& point, Map& map)
{
    std::function<bool(cv::Point2f&, cv::Point2f&, cv::Point2f&)> isInside =
        [&](cv::Point2f& a, cv::Point2f& b, cv::Point2f& c) {
            return ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) >= 0;
        };

    //如果点在多边形内部(包含边缘)，则势必满足点与所有边构成的叉乘向量大于等于0.这种情况是发生碰撞
    //只要有一例不符，则未发生碰撞
    for (Obstacle& obj : map.getObstacleList()) {
        bool collision = true;
        for (int i = 0; i < obj.getPointsSize(); ++i) {
            int j = (i == obj.getPointsSize() - 1) ? 0 : i + 1;
            if (!isInside(obj.getVertices()[i], obj.getVertices()[j], point)) {
                //只要有一个点不是inside，说明没有和这个物体发生碰撞
                collision = false;
                break;
            }
        }
        if (collision)
            return true;
    }
    return false;
}

bool checkCollisionPoint(cv::Point2f& point, Obstacle& obj)
{
    std::function<bool(cv::Point2f&, cv::Point2f&, cv::Point2f&)> isInside =
        [&](cv::Point2f& a, cv::Point2f& b, cv::Point2f& c) {
            return ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) >= 0;
        };
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

bool checkLineCollision(cv::Point2f p1, cv::Point2f p2, Map& map)
{
    //对所有障碍物进行check
    //如果点p1或点p2在障碍物内部，那么p1 p2的连线必然和障碍物发生碰撞
    if (checkCollisionPoint(p1, map) || checkCollisionPoint(p2, map))
        return true;
    for (auto& obj : map.getObstacleList()) {
        for (int i = 0; i < obj.getPointsSize(); ++i) {
            int j = (i == obj.getPointsSize() - 1) ? 0 : i + 1;
            if (checkTwoLineCollision(p1, p2, obj.getVertices()[i], obj.getVertices()[j]))
                return true;
        }
    }
    return false;
}

Obstacle createRectangle(const int width, const int height, cv::Point2i center)
{
    Obstacle obj;
    obj.push(center.x - width / 2, center.y - height / 2);
    obj.push(center.x + width / 2, center.y - height / 2);
    obj.push(center.x + width / 2, center.y + height / 2);
    obj.push(center.x - width / 2, center.y + height / 2);
    return obj;
}

cv::Point2i createCenterRandom(const int width, const int height, int win_width, int win_height)
{
    int maxL = max(width, height);
    win_width -= maxL;
    win_height -= maxL;
    float seed1 = rand() % (int)pow(10, 3) / (float)pow(10, 3);
    float seed2 = rand() % (int)pow(10, 3) / (float)pow(10, 3);
    cv::Point2i center;
    center.x = static_cast<int>(seed1 * win_width) + maxL / 2;
    center.y = static_cast<int>(seed2 * win_height) + maxL / 2;
    return center;
}

bool isValueObj(Obstacle& obj, Map& map, cv::Point2f& start, cv::Point2f& end)
{
    //不能和已加入障碍物碰撞(即重叠)
    for (int i = 0; i < obj.getPointsSize(); ++i) {
        int j = (i == obj.getPointsSize() - 1) ? 0 : i + 1;
        if (checkLineCollision(obj.getVertices()[i], obj.getVertices()[j], map))
            return false;
    }
    //不能直接覆盖了初始点和终点 否则无解
    return (!checkCollisionPoint(start, obj)) && (!checkCollisionPoint(end, obj));
}

void addObstaclesRandom(Map& map, cv::Point2f& start, cv::Point2f& end, int obsnum)
{
    srand(time(NULL));
    int rectangle_w1 = 50, rectangle_h1 = 50;
    int rectangle_w2 = 50, rectangle_h2 = 50;
    int rectangle_w3 = 100, rectangle_h3 = 100;
    int nums = 0;
    while (nums < obsnum) {
        cv::Point2i center1 =
            createCenterRandom(rectangle_w1, rectangle_h1, map.getWidth(), map.getHeight());
        cv::Point2i center2 =
            createCenterRandom(rectangle_w2, rectangle_h2, map.getWidth(), map.getHeight());
        cv::Point2i center3 =
            createCenterRandom(rectangle_w3, rectangle_h3, map.getWidth(), map.getHeight());
        Obstacle obj1 = createRectangle(rectangle_w1, rectangle_h1, center1);
        Obstacle obj2 = createRectangle(rectangle_w2, rectangle_h2, center2);
        Obstacle obj3 = createRectangle(rectangle_w3, rectangle_h3, center3);
        if (isValueObj(obj1, map, start, end)) {
            map.addObstacle(obj1);
            nums++;
        }
        if (isValueObj(obj2, map, start, end)) {
            map.addObstacle(obj2);
            nums++;
        }
        if (isValueObj(obj3, map, start, end)) {
            map.addObstacle(obj3);
            nums++;
        }
    }
}

Map createMapWithRandomObj(const std::string& window_name, const int width, const int height,
                           cv::Point2f& start, cv::Point2f& end)
{
    cv::Mat rrtImage(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    Map rrtMap(width, height, rrtImage, window_name);
    rrtMap.init();
    addObstaclesRandom(rrtMap, start, end, 30);
    // rrtMap.drawAllObstacles();
    return rrtMap;
}

Map createMapWithRandomObj2(const std::string& window_name, const int width, const int height,
                            cv::Point2f& start, cv::Point2f& end, int obsnum)
{
    cv::Mat rrtImage(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    Map rrtMap(width, height, rrtImage, window_name);
    rrtMap.init();
    addObstaclesRandom(rrtMap, start, end, obsnum);
    // rrtMap.drawAllObstacles();
    return rrtMap;
}

Map createMapWithObjList(const std::string& window_name, const int width, const int height,
                         vector<Obstacle>& objList)
{
    cv::Mat rrtImage(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    Map rrtMap(width, height, rrtImage, window_name);
    rrtMap.init();
    for (auto& obj : objList) {
        rrtMap.addObstacle(obj);
    }
    rrtMap.drawAllObstacles();
    return rrtMap;
}
