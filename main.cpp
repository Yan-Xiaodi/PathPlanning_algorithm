#include "map.h"
#include "rrt.h"

using namespace std;

int main(int argc, char** argv)
{
    const string window_name = "RRT Path Planning";
    //设置地图的宽和高
    const int width = 1000;
    const int height = 1000;
    const float stride = 10;        //扩展树的步长
    const float threashold = 0.1;   //分界概率值
    const int MaxItreation = 20000; //迭代上限
    const int rand_precition = 5;   //坐标精确位数

    //创建画布
    cv::Mat planMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //创建地图
    Map map(width, height, planMap, window_name);
    //地图初始化
    map.init();
    //绘制障碍物
    map.drawObstacles();

    // RRT算法
    //确定起始点和终点
    cv::Point2f start(0, 0), end(900, 900);
    RRT rrt(start, end, stride, threashold, MaxItreation, rand_precition, map);
    rrt.init();
    rrt.planning();
}
