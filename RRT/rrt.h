#ifndef RRT_H
#define RRT_H

#include "map.h"
#include "obstacle.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdlib.h>
#include <string>
#define eclipse 1e-4

class node
{
  public:
    node(float x_, float y_) : point(x_, y_), length(0), parent(nullptr) {}
    node(cv::Point2f p_) : point(p_), length(0), parent(nullptr) {}
    node(cv::Point2f p_, node* parent_) : point(p_), length(0), parent(parent_)
    {
        float dis =
            sqrt(pow(parent->point.x - point.x, 2) + pow(parent->point.y - point.y, 2));
        length = dis + parent->length;
        parent->childs[this] = dis;
    }

    void changeParent(node* new_parent)
    {
        parent->childs.erase(this);
        parent = new_parent;
        float dis =
            sqrt(pow(parent->point.x - point.x, 2) + pow(parent->point.y - point.y, 2));
        parent->childs[this] = dis;
        length = parent->length + dis;
    }

  public:
    cv::Point2f point;
    node* parent;                  //父节点
    float length;                  //路径代价(并不是最短)
    std::map<node*, float> childs; //{子节点，父节点到子节点的距离}
};

class RRT
{
  public:
    RRT() = default;
    RRT(cv::Point2f start_, cv::Point2f target_, float stride_, float threshold_,
        int MaxIteration_, int rand_precition_, Map* map_)
        : start(start_), target(target_), stride(stride_), threshold(threshold_),
          MaxIteration(MaxIteration_), rand_precition(rand_precition_), mapPtr(map_)
    {
    }

    void init();

    void planning();

    int getIteration();

    float getPlanTime();

    float getPlanLength();

    ~RRT();

  protected:
    float getDistance(cv::Point2f&, cv::Point2f&);

    int getNearestPoint(cv::Point2f&);

    cv::Point2f getSamplePoint();

    bool checkTwoLineCollision(cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f);

    bool checkLineCollision(cv::Point2f, cv::Point2f);

    bool checkCollision(cv::Point2f& point);

    bool findPath(cv::Point2f point);

  protected:
    cv::Point2f start;  //起点
    cv::Point2f target; //终点
    Map* mapPtr;
    float stride;                 //步长
    float threshold;              //概率分界
    int iter_cnt;                 //目前迭代次数
    int MaxIteration;             //最大迭代次数
    int rand_precition;           //随机数精度，小数点后的位数
    std::vector<node*> nodeLists; //随机扩展树的节点集合
    std::chrono::milliseconds timeval;
    float planLength; //最终规划的路径长度
};

#endif