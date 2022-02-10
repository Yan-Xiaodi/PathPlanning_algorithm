#include "rrt.h"
#include "rrt_star.h"

using namespace std;

void SetObstacles(Map& map)
{

    Obstacle obj1, obj2;
    obj1.push(50, 50);
    obj1.push(300, 50);
    obj1.push(300, 300);
    obj1.push(50, 300);
    map.addObstacle(obj1);

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < obj1.getPointsSize(); ++j) {
            obj1.setVertices()[j].x += 300;
        }
        map.addObstacle(obj1);
    }

    obj2.push(80, 500);
    obj2.push(130, 500);
    obj2.push(130, 900);
    obj2.push(80, 900);
    map.addObstacle(obj2);
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < obj2.getPointsSize(); ++j) {
            obj2.setVertices()[j].x += 150;
        }
        map.addObstacle(obj2);
    }
}

void rrt_compare_with_rrtstar(const string& win1, const string& win2, const int& width,
                              const int& height, const float& stride,
                              const float& threashold, const int& MaxIteration,
                              const int& rand_precition, const float& radius,
                              cv::Point2f& start, cv::Point2f& end)
{
    cv::Mat rrtImage(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    Map rrtMap(width, height, rrtImage, win1);
    rrtMap.init();
    SetObstacles(rrtMap);
    rrtMap.drawAllObstacles();

    cv::Mat rrtstarImage(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    Map rrtstarMap(width, height, rrtstarImage, win2);
    rrtstarMap.init();
    SetObstacles(rrtstarMap);
    rrtstarMap.drawAllObstacles();

    // rrt算法
    RRT rrt(start, end, stride, threashold, MaxIteration, rand_precition, rrtMap);
    rrt.init();
    rrt.planning();
    const string iteration1 = "itreation: " + to_string(rrt.getIteration());
    const string time1 = "time cost: " + to_string(rrt.getPlanTime());
    const string length1 = "plan length: " + to_string(rrt.getPlanLength());
    int w = 20, h = 910;
    rrtMap.drawText(iteration1, cv::Point(w, h));
    rrtMap.drawText(time1, cv::Point(w, h + 30));
    rrtMap.drawText(length1, cv::Point(w, h + 60));

    // rrt* 算法
    RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition,
                      rrtstarMap, radius);
    rrt_star.init();
    rrt_star.planning();
    const string iteration2 = "itreation: " + to_string(rrt_star.getIteration());
    const string time2 = "time cost: " + to_string(rrt_star.getPlanTime());
    const string length2 = "plan length: " + to_string(rrt_star.getPlanLength());
    rrtstarMap.drawText(iteration2, cv::Point(w, h));
    rrtstarMap.drawText(time2, cv::Point(w, h + 30));
    rrtstarMap.drawText(length2, cv::Point(w, h + 60));

    cv::imshow(win1, rrtMap.getPlanMap());
    cv::imshow(win2, rrtstarMap.getPlanMap());
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    const string window_name1 = "RRT Path Planning";
    const string window_name2 = "RRT* Path Planning";
    //设置地图的宽和高
    const int width = 1000;
    const int height = 1000;
    const float stride = 10;        //扩展树的步长
    const float threashold = 0.05;  //分界概率值
    const int MaxIteration = 20000; //迭代上限
    const int rand_precition = 5;   //坐标精确位数
    cv::Point2f start(0, 0), end(900, 900);
    float radius = 50;

    /*
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
    RRT_STAR rrt_star(start, end, stride, threashold, MaxItereation, rand_precition, map,
                      radius);
    rrt_star.init();
    rrt_star.planning();
    */
    rrt_compare_with_rrtstar(window_name1, window_name2, width, height, stride,
                             threashold, MaxIteration, rand_precition, radius, start,
                             end);
}
