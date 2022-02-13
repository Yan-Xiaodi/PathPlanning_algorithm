#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "rrt.h"
#include <algorithm>

class RRT_STAR : public RRT
{
  public:
    RRT_STAR(cv::Point2f start_, cv::Point2f target_, int stride_, float threshold_,
             int MaxIteration_, int rand_precition_, Map* map_, float radius_)
        : RRT(start_, target_, stride_, threshold_, MaxIteration_, rand_precition_, map_),
          radius(radius_)
    {
    }

    void planning();

  private:
    void changeParent(node*, node*, float);

    //重选父节点
    bool ReselectParentNode(node*);

    //重布线随机树
    void ReWrite(node*);

  private:
    float radius;
};

#endif