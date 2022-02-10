#include "rrt.h"

void RRT::init()
{
    //将起点加入到点集中
    node* n = new node(start);
    nodeLists.push_back(n);
    map.drawPoint(start);
    srand(time(NULL)); //设置随机数种子，使每次产生的随机序列不同
    //绘制起始点和终点
    cv::circle(map.getPlanMap(), cv::Point(start.x, start.y), 5, cv::Scalar(0, 0, 255),
               -1);
    cv::circle(map.getPlanMap(), cv::Point(target.x, target.y), 5, cv::Scalar(0, 0, 255),
               -1);
}

int RRT::getNearestPoint(cv::Point2f& q_rand)
{
    int index = 0;
    float minDistance = std::numeric_limits<float>::max();
    for (int i = 0; i < nodeLists.size(); ++i) {
        float dis = pow(q_rand.x - nodeLists[i]->point.x, 2) +
                    pow(q_rand.y - nodeLists[i]->point.y, 2);
        if (dis < minDistance) {
            minDistance = dis;
            index = i;
        }
    }
    return index;
}

cv::Point2f RRT::getSamplePoint()
{
    float seed = rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);

    if (seed < threshold) {
        return target;
    } else {
        float x_rand =
            rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
        float y_rand =
            rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
        return cv::Point2f(x_rand * map.getWidth(), y_rand * map.getHeight());
    }
}

bool RRT::checkTwoLineCollision(cv::Point2f p1, cv::Point2f p2, cv::Point2f q1,
                                cv::Point2f q2)
{
    //判断线段是否相交的方法：
    //若两个线段分别投影到xy轴上，存在 一个轴上两个线段的区间没有交集的情况 则为不想交

    //确定区间
    std::pair<float, float> x1(std::max(p1.x, p2.x), std::min(p1.x, p2.x));
    std::pair<float, float> y1(std::max(p1.y, p2.y), std::min(p1.y, p2.y));
    std::pair<float, float> x2(std::max(q1.x, q2.x), std::min(q1.x, q2.x));
    std::pair<float, float> y2(std::max(q1.y, q2.y), std::min(q1.y, q2.y));

    int collision = 0;
    //判断区间是否相交
    auto compare = [](std::pair<float, float> a, std::pair<float, float> b) {
        if (a.first < b.first)
            swap(a, b);
        return b.first >= a.second;
    };

    if (compare(x1, x2) && compare(y1, y2))
        return true;
    return false;
}

bool RRT::checkLineCollision(cv::Point2f p1, cv::Point2f p2)
{
    //对所有障碍物进行check
    //如果点p1或点p2在障碍物内部，那么p1 p2的连线必然和障碍物发生碰撞
    if (checkCollision(p1) || checkCollision(p2))
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

bool RRT::checkCollision(cv::Point2f& point)
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

bool RRT::findPath(cv::Point2f point)
{
    float dis = sqrt(pow(point.x - target.x, 2) + pow(point.y - target.y, 2));
    //如果q_new点到target的距离小于步长stride，说明找到了目标点target(可以用q_new到target的直线连接)
    if (dis <= stride)
        return true;
    return false;
}

void RRT::planning()
{
    iter_cnt = 0;
    bool isfind = false;
    std::chrono::high_resolution_clock::time_point begin, end;
    begin = std::chrono::high_resolution_clock::now();

    while (iter_cnt++ < MaxIteration) {
        cv::Point2f q_rand = getSamplePoint();
        int nr_index = getNearestPoint(q_rand);
        cv::Point2f q_nearest = nodeLists[nr_index]->point;
        //获取q_new
        float rate = stride / sqrt(pow(q_rand.x - q_nearest.x, 2) +
                                   pow(q_rand.y - q_nearest.y, 2));
        cv::Point2f q_new(q_nearest.x + (q_rand.x - q_nearest.x) * rate,
                          q_nearest.y + (q_rand.y - q_nearest.y) * rate);
        //节点没有和障碍物发生碰撞，且连线也没和障碍物碰撞，就将新的节点加入到扩展树中
        if (!checkLineCollision(q_new, nodeLists[nr_index]->point)) {
            //没有和障碍物发生碰撞，就将新的节点加入到扩展树中
            node* n_new = new node(q_new, nodeLists[nr_index]);
            nodeLists.push_back(n_new);

            //绘制节点和连接的路径
            map.drawPoint(q_new);
            map.drawLine(q_new, nodeLists[nr_index]->point);
            //判断是否达到终点
            if (findPath(q_new)) {
                isfind = true;
                node* n_end = new node(target, n_new);
                nodeLists.push_back(n_end);
                map.drawLine(q_new, target);
            }
        }
        //显示图像
        cv::imshow(map.getWindowName(), map.getPlanMap());
        cv::waitKey(2);
        //找到路径 退出迭代循环
        if (isfind) {
            end = std::chrono::high_resolution_clock::now();
            break;
        }
    }
    //绘制路径
    if (isfind) {
        node* n = nodeLists.back();
        float len = n->length;
        cv::Mat image = map.getPlanMap();
        while (n != nullptr && n->parent != nullptr) {
            cv::line(image, cv::Point(n->point.x, n->point.y),
                     cv::Point(n->parent->point.x, n->parent->point.y),
                     cv::Scalar(0, 0, 255), 2);
            n = n->parent;
            cv::imshow(map.getWindowName(), image);
            cv::waitKey(2);
        }
        timeval = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        printf("Path has been fount,it's length is %.2f,and it cost %.3f seconds and %d "
               "iteration to find the path\n ",
               len, timeval.count() / (float)1000, iter_cnt);
    } else {
        printf("Paht has not been found,may be you should increase the MaxInteration\n");
    }
}

int RRT::getIteration() { return iter_cnt; }

float RRT::getPlanTime() { return timeval.count() / (float)1000; }

float RRT::getPlanLength() { return nodeLists.back()->length; }

RRT::~RRT()
{
    for (auto& n : nodeLists)
        delete n;
}
