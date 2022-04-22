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
                 int MaxIteration_, int rand_precition_, Map* map_, float rate)
        : RRT_Connect(start_, target_, stride_, threshold_, MaxIteration_, rand_precition_, map_),
          growRate(rate)
    {
    }

    void planning()
    {
        iter_cnt = 0;
        bool isfind = false;
        bool reverse = false;
        timeBegin = std::chrono::high_resolution_clock::now();
        selfplanning(nodeLists, nodeLists2, iter_cnt, isfind, reverse);
        timeval = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeBegin);
        if (isfind) {
            printf("GRRT_Connect algorithm: Path has been fount,it's length is %.2f,and "
                   "it cost %.3f seconds and %d iteration to find the path\n ",
                   planLength, timeval.count() / (float)1000, iter_cnt);
        } else {
            printf("GRRT_Connect algorithm:Paht has not been found,may be you should "
                   "increase the MaxInteration\n");
        }
    }

  protected:
    void selfplanning(std::vector<node*>& Tree1, std::vector<node*>& Tree2, int& iter, bool& isfind,
                      bool reverse)
    {
        if (iter > MaxIteration || isfind)
            return;
        cv::Point2f q_rand = getSamplePoint(Tree1);
        int nr_index = getNearestPoint(q_rand, Tree1);
        cv::Point2f q_nearest = Tree1[nr_index]->point;
        //获取q_new
        float rate = stride / getDistance(q_rand, q_nearest);
        cv::Point2f q_new(q_nearest.x + (q_rand.x - q_nearest.x) * rate,
                          q_nearest.y + (q_rand.y - q_nearest.y) * rate);

        //节点没有和障碍物发生碰撞，且连线也没和障碍物碰撞，就将新的节点加入到扩展树中
        if (!checkLineCollision(q_new, q_nearest)) {
            node* n_new = new node(q_new, Tree1[nr_index]);
            Tree1.push_back(n_new);

            //绘制节点和连接的路径
            mapPtr->drawPoint(q_new);

            if (!reverse)
                mapPtr->drawLineColor(q_new, q_nearest, cv::Scalar(0, 255, 0));
            else
                mapPtr->drawLineColor(q_new, q_nearest, cv::Scalar(255, 0, 0));
            //显示图像
            mapPtr->show();

            //将q_new作为Tree2 的目标节点
            int nr_index2 = getNearestPoint(q_new, Tree2);
            cv::Point2f q_nearest2 = Tree2[nr_index2]->point;
            // Tree2进行贪婪扩展：如果不发生碰撞就让q_new2不断向Tree1的q_new方向进行延伸
            node* q_tmp = Tree2[nr_index2];

            //按growRate倍增的步长
            float growStride = stride;
            while (iter++ < MaxIteration) {
                //获取q_new2
                growStride *= growRate;
                printf("%f\n", growStride);
                float rate2 = growStride / getDistance(q_new, q_nearest2);
                cv::Point2f q_new2(q_nearest2.x + (q_new.x - q_nearest2.x) * rate2,
                                   q_nearest2.y + (q_new.y - q_nearest2.y) * rate2);

                if (checkLineCollision(q_new2, q_nearest2)) {
                    //如果发生碰撞，就要退出循环
                    break;
                }
                node* n_new2 = new node(q_new2, q_tmp);
                Tree2.push_back(n_new2);

                //绘制节点和连接的路径
                mapPtr->drawPoint(q_new2);

                if (reverse)
                    mapPtr->drawLineColor(q_new2, q_nearest2, cv::Scalar(0, 255, 0));
                else
                    mapPtr->drawLineColor(q_new2, q_nearest2, cv::Scalar(255, 0, 0));
                mapPtr->show();

                //判断q_new2是否到达q_new附近
                if (getDistance(q_new2, q_new) <= stride) {

                    timeEnd = std::chrono::high_resolution_clock::now();
                    //绘制路径
                    if (reverse)
                        mapPtr->drawLineColor(q_new2, q_nearest2, cv::Scalar(0, 255, 0));
                    else
                        mapPtr->drawLineColor(q_new2, q_nearest2, cv::Scalar(255, 0, 0));

                    mapPtr->drawLineColor(q_new2, q_new, cv::Scalar(0, 0, 255));

                    std::map<int, node*> path1 = getPath(n_new);
                    std::map<int, node*> path2 = getPath(n_new2);
                    //贪心二分剪枝
                    binaryPurgeNode(path1, iter);
                    binaryPurgeNode(path2, iter);

                    planLength = getDistance(q_new2, q_new) + n_new->length + n_new2->length;

                    while (n_new != nullptr && n_new->parent != nullptr) {
                        mapPtr->drawLineColor(n_new->point, n_new->parent->point,
                                              cv::Scalar(0, 0, 255));
                        n_new = n_new->parent;
                        mapPtr->show();
                    }

                    while (n_new2 != nullptr && n_new2->parent != nullptr) {
                        mapPtr->drawLineColor(n_new2->point, n_new2->parent->point,
                                              cv::Scalar(0, 0, 255));
                        n_new2 = n_new2->parent;
                        mapPtr->show();
                    }
                    isfind = true;
                    break;
                }
                //替换
                q_nearest2 = q_new2;
                q_tmp = n_new2;
            }
        }
        if (isfind)
            return;
        //选择节点数量少的那一棵树作为作为Tree1,优先扩展
        if (Tree2.size() > Tree1.size())
            selfplanning(Tree1, Tree2, iter, isfind, reverse);
        else
            selfplanning(Tree2, Tree1, iter, isfind, !reverse);
    }

    std::map<int, node*> getPath(node* lastNode)
    {
        int index = 0;
        std::map<int, node*> path;
        while (lastNode != nullptr) {
            path[index] = lastNode;
            lastNode = lastNode->parent;
            index++;
        }
        return path;
    }

    void binaryPurgeNode(std::map<int, node*>& path, int& iter)
    {
        int low = path.begin()->first, high = (--path.end())->first;
        /**
         * low 表示路径的最终节点，high表示起始点
         *首先将path[low]作为终点target，path[high]作为起始点start
         *二分法剪枝就是以起始点start开始，判断是否能和目标节点target直接相连
         *如果可以就将start设置为target，target重新设置为path[high];如果不行则将target设置为(target+start)/2
         *二分剪枝结束的判断：low>=high 表示路径已全部优化完毕
         **/
        while (low < high) {
            int mid = low;
            while ((mid + 1) < high) {
                iter++;
                node* start = path[high];
                node* target = path[mid];
                if (!checkLineCollision(target->point, start->point)) {
                    target->changeParent(start);
                    high = mid;
                } else {
                    mid = (mid + high) / 2;
                }
                printf("%d\n", iter);
            }
            high = mid;
        }
    }

  private:
    float growRate;
};

#endif