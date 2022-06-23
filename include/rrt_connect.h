#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "rrt.h"

class RRT_Connect : public RRT
{
  public:
    RRT_Connect(cv::Point2f start_, cv::Point2f target_, int stride_, float threshold_,
                int MaxIteration_, int rand_precition_, Map* map_)
        : RRT(start_, target_, stride_, threshold_, MaxIteration_, rand_precition_, map_)
    {
    }
    void init();
    void planning();

    ~RRT_Connect();

  protected:
    void selfplanning(std::vector<node*>& Tree1, std::vector<node*>& Tree2, int& iter,
                      bool& isfind, bool reverse);

    int getNearestPoint(cv::Point2f& p, std::vector<node*>& Tree);

    cv::Point2f getSamplePoint(std::vector<node*>& Tree);

  protected:
    std::vector<node*> nodeLists2; //随机扩展树2
    std::chrono::high_resolution_clock::time_point timeBegin, timeEnd;
};

#endif