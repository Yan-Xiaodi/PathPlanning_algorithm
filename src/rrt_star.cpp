#include "rrt_star.h"

void RRT_STAR::changeParent(node* n, node* n_parent, float dis)
{
    //断开和原来的连接
    n->parent->childs.erase(n);
    //将这个新节点作为parent
    n->parent = n_parent;
    n_parent->childs[n] = dis;
    n->length = n_parent->length + dis;
}

//重选父节点
bool RRT_STAR::ReselectParentNode(node* q_new)
{
    //遍历所有节点，筛选出半径radius邻域内的所有节点,比较路径代价
    int index = -1;
    float minDis = q_new->length;
    float len = 0;

    for (int i = 0; i < nodeLists.size(); ++i) {
        node* q_i = nodeLists[i];
        float dis = getDistance(q_new->point, q_i->point);
        if ((dis < radius) && ((dis + q_i->length) < minDis)) {
            // q_new和被选中的节点之间的连线line不和障碍物发生碰撞
            if (!checkLineCollision(q_new->point, q_i->point)) {
                index = i;
                minDis = dis + q_i->length;
                len = dis;
            }
        }
    }
    if (index != -1) {
        changeParent(q_new, nodeLists[index], len);
        iter_cnt++;
        return true;
    }
    return false;
}

//重布线随机树
void RRT_STAR::ReWrite(node* q_new)
{
    for (int i = 0; i < nodeLists.size(); ++i) {
        node* q_i = nodeLists[i];
        float dis = getDistance(q_new->point, q_i->point);
        // 若q_i节点将父节点更换为q_new后取得更小的路径代价 且更换后的连线不发生碰撞
        // 则更换父节点
        if ((dis < radius) && (q_i->length > (q_new->length + dis)) &&
            (!checkLineCollision(q_i->point, q_new->point))) {
            changeParent(q_i, q_new, dis);

            //使用蓝线连接q_i和q_new
            mapPtr->drawLineColor(q_i->point, q_new->point, cv::Scalar(255, 0, 0));

            // q_i重新选择父节点后需要将它的所有子节点的路径代价length进行更新
            std::function<void(node*)> updateChilds = [&](node* n) {
                if (n == nullptr || n->childs.empty())
                    return;
                for (auto& [child, len] : n->childs) {
                    child->length = child->parent->length + len;
                    updateChilds(child);
                }
            };
            updateChilds(q_i);
            iter_cnt++;
        }
    }
}

void RRT_STAR::planning()
{
    bool isfind = false;
    std::chrono::high_resolution_clock::time_point begin, end;
    begin = std::chrono::high_resolution_clock::now();

    while (iter_cnt++ < MaxIteration) {
        cv::Point2f q_rand = getSamplePoint();
        int nr_index = getNearestPoint(q_rand);
        cv::Point2f q_nearest = nodeLists[nr_index]->point;
        //获取q_new
        float rate = stride / getDistance(q_rand, q_nearest);
        cv::Point2f q_new(q_nearest.x + (q_rand.x - q_nearest.x) * rate,
                          q_nearest.y + (q_rand.y - q_nearest.y) * rate);
        //判断是否发生碰撞
        if (!checkLineCollision(q_new, q_nearest)) {
            node* n_new = new node(q_new, nodeLists[nr_index]);

            //重选父节点
            ReselectParentNode(n_new);
            nodeLists.push_back(n_new);
            //重布线随机树
            ReWrite(n_new);

            //绘制节点和连接的路径
            mapPtr->drawPoint(n_new->point);
            mapPtr->drawLine(n_new->point, n_new->parent->point);

            //判断是否达到终点
            if (findPath(n_new->point)) {
                isfind = true;
                node* n_end = new node(target, n_new);
                nodeLists.push_back(n_end);
                mapPtr->drawLine(n_new->point, target);
            }
        }
        //显示图像
        mapPtr->show();
        //找到路径 退出迭代循环
        if (isfind) {
            end = std::chrono::high_resolution_clock::now();
            break;
        }
    }
    //绘制路径
    if (isfind) {
        node* n = nodeLists.back();
        planLength = n->length;
        while (n != nullptr && n->parent != nullptr) {
            mapPtr->drawLineColor(n->point, n->parent->point, cv::Scalar(0, 0, 255));
            n = n->parent;
            mapPtr->show();
        }
        timeval = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        printf("RRT_star algorithm:Path has been fount,it's length is %.2f,and it cost "
               "%.3f seconds and %d "
               "iteration to find the path\n ",
               planLength, timeval.count() / (float)1000, iter_cnt);
    } else {
        printf("RRT_star algorithm:Paht has not been found,may be you should increase "
               "the MaxInteration\n");
    }
}