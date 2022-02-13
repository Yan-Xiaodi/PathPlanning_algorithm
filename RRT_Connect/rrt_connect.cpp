#include "rrt_connect.h"

void RRT_Connect::init()
{
    //将起点加入到Tree1中
    node* n_start = new node(start);
    nodeLists.push_back(n_start);

    //终点加入Tree2中
    node* n_target = new node(target);
    nodeLists2.push_back(n_target);

    srand(time(NULL)); //设置随机数种子，使每次产生的随机序列不同
    //绘制起始点和终点
    cv::circle(mapPtr->getPlanMap(), cv::Point(start.x, start.y), 5,
               cv::Scalar(0, 0, 255), -1);
    cv::circle(mapPtr->getPlanMap(), cv::Point(target.x, target.y), 5,
               cv::Scalar(0, 0, 255), -1);
}
void RRT_Connect::planning()
{
    iter_cnt = 0;
    bool isfind = false;
    bool reverse = false;
    timeBegin = std::chrono::high_resolution_clock::now();
    selfplanning(nodeLists, nodeLists2, iter_cnt, isfind, reverse);
    timeval = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeBegin);
    if (isfind) {
        printf("RRT_Connect algorithm: Path has been fount,it's length is %.2f,and "
               "it cost %.3f seconds and %d iteration to find the path\n ",
               planLength, timeval.count() / (float)1000, iter_cnt);
    } else {
        printf("RRT_Connect algorithm:Paht has not been found,may be you should "
               "increase the MaxInteration\n");
    }
}

void RRT_Connect::selfplanning(std::vector<node*>& Tree1, std::vector<node*>& Tree2,
                               int& iter, bool& isfind, bool reverse)
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

        while (iter++ < MaxIteration) {
            //获取q_new2
            float rate2 = stride / getDistance(q_new, q_nearest2);
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

            //判断q_new2是否到达q_new附近
            if (getDistance(q_new2, q_new) <= stride) {

                timeEnd = std::chrono::high_resolution_clock::now();
                //绘制路径
                if (reverse)
                    mapPtr->drawLineColor(q_new2, q_nearest2, cv::Scalar(0, 255, 0));
                else
                    mapPtr->drawLineColor(q_new2, q_nearest2, cv::Scalar(255, 0, 0));
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

int RRT_Connect::getNearestPoint(cv::Point2f& p, std::vector<node*>& Tree)
{
    int index = 0;
    float minDistance = std::numeric_limits<float>::max();
    for (int i = 0; i < Tree.size(); ++i) {
        float dis = getDistance(p, Tree[i]->point);
        if (dis < minDistance) {
            minDistance = dis;
            index = i;
        }
    }
    return index;
}

cv::Point2f RRT_Connect::getSamplePoint(std::vector<node*>& Tree)
{
    float seed = rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);

    if (seed < threshold) {
        if (Tree == nodeLists)
            return target;
        else
            return start;
    } else {
        float x_rand =
            rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
        float y_rand =
            rand() % (int)pow(10, rand_precition) / (float)pow(10, rand_precition);
        return cv::Point2f(x_rand * mapPtr->getWidth(), y_rand * mapPtr->getHeight());
    }
}

RRT_Connect::~RRT_Connect()
{
    for (auto& n : nodeLists2)
        delete n;
}