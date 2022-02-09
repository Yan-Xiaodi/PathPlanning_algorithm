#ifndef RRT_H
#define RRT_H

#include "obstacle.h"
#include <cmath>
#include <functional>
#include <limits>
#include <stdlib.h>
#include <time.h>
#define eclipse 1e-6
class node
{
  public:
    node(float x_, float y_) : point(x_, y_), length(0), parent(nullptr) {}
    node(cv::Point2f p_) : point(p_), length(0), parent(nullptr) {}
    node(cv::Point2f p_, node* parent_) : point(p_), length(0), parent(parent_)
    {
        length =
            sqrt(pow(parent->getX() - point.x, 2) + pow(parent->getY() - point.y, 2)) +
            parent->getLength();
    }

    float getLength() { return length; }
    float getX() { return point.x; }
    float getY() { return point.y; }
    cv::Point2f getPoint() { return point; }

  private:
    cv::Point2f point;
    node* parent; //父节点
    float length; //到根节点的距离(并不是最短)
};

class RRT
{
  public:
    RRT() = default;
    RRT(cv::Point2f start_, cv::Point2f target_, int stride_, float threshold_,
        int MaxIteration_, int rand_precition_, std::vector<Obstacle> obstacles_)
        : start(start_), target(target_), stride(stride_), threshold(threshold_),
          MaxIteration(MaxIteration_), rand_precition(rand_precition_),
          obstacles(obstacles_)
    {
    }

    void addNode(cv::Point2f point, node* parent)
    {
        if (parent != nullptr)
            node* n = new node(point, parent);
        else
            node* n = new node(point);
    }

    void init(int width, int height)
    {
        delta_time = 0.0;
        //将起点加入到点集中
        node* n = new node(start);
        nodeLists.push_back(n);
        map_x = width;
        map_y = height;
    }

    int getNearestPoint(cv::Point2f& q_rand)
    {
        int index = 0;
        float minDistance = std::numeric_limits<float>::max();
        for (int i = 0; i < nodeLists.size(); ++i) {
            float dis = pow(q_rand.x - nodeLists[i]->getX(), 2) +
                        pow(q_rand.y - nodeLists[i]->getY(), 2);
            if (dis < minDistance) {
                minDistance = dis;
                index = i;
            }
        }
        return index;
    }

    cv::Point2f getSamplePoint()
    {
        srand(time(NULL)); //设置随机数种子，使每次产生的随机序列不同
        float seed =
            rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
        if (seed < threshold)
            return target;
        else {
            float x_rand =
                rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
            float y_rand =
                rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
            return cv::Point2f(x_rand * map_x, y_rand * map_y);
        }
    }

    bool isCollision(cv::Point2f& point)
    {
        std::function<bool(cv::Point2f&, cv::Point2f&, cv::Point2f&)> isInside =
            [&](cv::Point2f& a, cv::Point2f& b, cv::Point2f& c) {
                return ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) >= 0;
            };

        //如果点在多边形内部(包含边缘)，则势必满足点与所有边构成的叉乘向量大于等于0.这种情况是发生碰撞
        //只要有一例不符，则未发生碰撞
        for (Obstacle& obj : obstacles) {
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

    bool findPath(cv::Point2f point)
    {
        float dis = pow(point.x - target.x, 2) + pow(point.y - target.y, 2);
        if (dis < eclipse)
            return true;
    }

    void planning()
    {
        int iter_cnt = 0;
        bool isfind = false;
        while (iter_cnt++ < MaxIteration) {
            cv::Point2f q_rand = getSamplePoint();
            int nr_index = getNearestPoint(q_rand);
            cv::Point2f q_nearest = nodeLists[nr_index]->getPoint();
            //达到终点
            if (findPath(q_nearest)) {
                isfind = true;
                break;
            }
            //获取q_new
            float rate = threshold / sqrt(pow(q_rand.x - q_nearest.x, 2) +
                                          pow(q_rand.y - q_nearest.y, 2));
            cv::Point2f q_new(q_nearest.x + (q_rand.x - q_nearest.x) * rate,
                              q_nearest.y + (q_rand.y - q_nearest.y) * rate);
            if (!isCollision(q_new)) {
                //没有和障碍物发生碰撞，就将新的节点加入到扩展树中
                node* n_new = new node(q_new, nodeLists[nr_index]);
                nodeLists.push_back(n_new);

                //绘制节点和连接的路径
            }
        }
    }

    ~RRT()
    {
        for (auto& n : nodeLists)
            delete n;
    }

  private:
    cv::Point2f start;               //起点
    cv::Point2f target;              //终点
    int map_x;                       //地图的长(x)
    int map_y;                       //地图的高(y)
    int stride;                      //步长
    float threshold;                 //概率分界
    int MaxIteration;                //最大迭代次数
    double delta_time;               //规划时间
    int rand_precition;              //随机数精度，小数点后的位数
    std::vector<Obstacle> obstacles; //障碍物集合
    std::vector<node*> nodeLists;    //随机扩展树的节点集合
};

#endif