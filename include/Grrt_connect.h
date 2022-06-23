/**
 * Algorithm: GreedyRRT-Connect
 * Author   : Yan-XiaoDi
 * brief introduction:
 *     Base to RRT-Connect,add two Greedy strategy:
 *     1.When tree is greedy growing,let stride greedy growing too,by multiply growRate
 *each time, until collision with obstacle,then stride turn  to  the initial value. 2.When
 *find the path from start to goal,use a greedy way based on dichotomy to pruning the node
 *     on the path,it can delete those unnecessary node,reduce the cost.And it is little
 *time cost
 **/

#ifndef GRRT_CONNECT_H
#define GRRT_CONNECT_H

#include "rrt_connect.h"

class GRRT_Connect : public RRT_Connect
{
  public:
    GRRT_Connect(cv::Point2f start_, cv::Point2f target_, int stride_, float threshold_,
                 int MaxIteration_, int rand_precition_, Map* map_, float rate, float dis)
        : RRT_Connect(start_, target_, stride_, threshold_, MaxIteration_, rand_precition_, map_),
          growRate(rate), dismin(dis)
    {
    }

    void planning();

  protected:
    void selfplanning(std::vector<node*>& Tree1, std::vector<node*>& Tree2, int& iter, bool& isfind,
                      bool reverse);

    std::map<int, node*> getPath(node* lastNode);

    void binaryPurgeNode(std::map<int, node*>& path, int& iter);

  private:
    float growRate;
    float dismin;
};

#endif