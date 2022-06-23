#include "Grrt_connect.h"
#include "rrt.h"
#include "rrt_connect.h"
#include "rrt_star.h"
#include "utils.h"
#include <fstream>

using namespace std;
ofstream rrtRecord("rrt.txt", ios::out);
ofstream rrtStarRecord("rrt_star.txt", ios::out);
ofstream rrtConnectRecord("rrt_connect.txt", ios::out);
ofstream GrrtConnectRecord("grrt_connect.txt", ios::out);

vector<float> rrtTime;
vector<float> rrtLength;

vector<float> rrtStarTime;
vector<float> rrtStarLength;

vector<float> rrtConnectTime;
vector<float> rrtConnectLength;

vector<float> GrrtConnectTime;
vector<float> GrrtConnectLength;

void solve(const std::string& plannerName, Map* mapPtr, const float& stride,
           const float& threashold, const int& MaxIteration, const int& rand_precition,
           const float& radius, const float& grow_rate, const float& dismin, cv::Point2f& start,
           cv::Point2f& end)
{
    RRT* planner;
    if (plannerName == "rrt") {
        RRT rrt(start, end, stride, threashold, MaxIteration, rand_precition, mapPtr);
        rrt.init();
        rrt.planning();
        planner = &rrt;
        rrtTime.push_back(rrt.getPlanTime());
        rrtLength.push_back(rrt.getPlanLength());
    } else if (plannerName == "rrt_star") {
        RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition, mapPtr,
                          radius);
        rrt_star.init();
        rrt_star.planning();
        planner = &rrt_star;
        rrtStarTime.push_back(rrt_star.getPlanTime());
        rrtStarLength.push_back(rrt_star.getPlanLength());
    } else if (plannerName == "rrt_connect") {
        RRT_Connect rrt_connect(start, end, stride, threashold, MaxIteration, rand_precition,
                                mapPtr);
        rrt_connect.init();
        rrt_connect.planning();
        planner = &rrt_connect;
        rrtConnectTime.push_back(rrt_connect.getPlanTime());
        rrtConnectLength.push_back(rrt_connect.getPlanLength());
    } else if (plannerName == "Grrt_connect") {
        GRRT_Connect Grrt_connect(start, end, stride, threashold, MaxIteration, rand_precition,
                                  mapPtr, grow_rate, dismin);
        Grrt_connect.init();
        Grrt_connect.planning();
        planner = &Grrt_connect;
        GrrtConnectTime.push_back(Grrt_connect.getPlanTime());
        GrrtConnectLength.push_back(Grrt_connect.getPlanLength());
    }
    // const string iteration3 = "itreation: " + to_string(planner->getIteration());
    // const string time3 = "time cost: " + to_string(planner->getPlanTime());
    // const string length3 = "plan length: " + to_string(planner->getPlanLength());
    // mapPtr->drawText(iteration3, cv::Point(20, 920));
    // mapPtr->drawText(time3, cv::Point(20, 950));
    // mapPtr->drawText(length3, cv::Point(20, 980));
}

void compare_planner(const string& win1, const string& win2, const string& win3, const string& win4,
                     const int& width, const int& height, const float& stride,
                     const float& threashold, const int& MaxIteration, const int& rand_precition,
                     const float& radius, const float& grow_rate, const float& dismin,
                     cv::Point2f& start, cv::Point2f& end)
{
    Map tmp = createMapWithRandomObj(win1, width, height, start, end);
    vector<Obstacle> objList = tmp.getObstacleList();
    Map map1 = createMapWithObjList(win1, width, height, objList);
    Map map2 = createMapWithObjList(win2, width, height, objList);
    Map map3 = createMapWithObjList(win3, width, height, objList);
    Map map4 = createMapWithObjList(win4, width, height, objList);

    solve("rrt", &map1, stride, threashold, MaxIteration, rand_precition, radius, grow_rate, dismin,
          start, end);
    solve("rrt_star", &map2, stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
          dismin, start, end);
    solve("rrt_connect", &map3, stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
          dismin, start, end);
    solve("Grrt_connect", &map4, stride, threashold, MaxIteration, rand_precition, radius,
          grow_rate, dismin, start, end);

    cv::imshow(win1, map1.getPlanMap());
    cv::imshow(win2, map2.getPlanMap());
    cv::imshow(win3, map3.getPlanMap());
    cv::imshow(win4, map4.getPlanMap());
    cv::imwrite(win1 + ".png", map1.getPlanMap());
    cv::imwrite(win2 + ".png", map2.getPlanMap());
    cv::imwrite(win3 + ".png", map3.getPlanMap());
    cv::imwrite(win4 + ".png", map4.getPlanMap());
    cv::waitKey(0);
}

void compare_planner2(const string& win1, const string& win2, const string& win3,
                      const string& win4, const int& width, const int& height, const float& stride,
                      const float& threashold, const int& MaxIteration, const int& rand_precition,
                      const float& radius, const float& grow_rate, const float& dismin,
                      cv::Point2f& start, cv::Point2f& end, int obsnum)
{
    Map tmp = createMapWithRandomObj2(win1, width, height, start, end, obsnum);
    vector<Obstacle> objList = tmp.getObstacleList();
    Map map1 = createMapWithObjList(win1, width, height, objList);
    Map map2 = createMapWithObjList(win2, width, height, objList);
    Map map3 = createMapWithObjList(win3, width, height, objList);
    Map map4 = createMapWithObjList(win4, width, height, objList);

    solve("rrt", &map1, stride, threashold, MaxIteration, rand_precition, radius, grow_rate, dismin,
          start, end);
    solve("rrt_star", &map2, stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
          dismin, start, end);
    solve("rrt_connect", &map3, stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
          dismin, start, end);
    solve("Grrt_connect", &map4, stride, threashold, MaxIteration, rand_precition, radius,
          grow_rate, dismin, start, end);
}

int main(int argc, char** argv)
{
    const string window_name1 = "RRT Path Planning";
    const string window_name2 = "RRT* Path Planning";
    const string window_name3 = "RRT-Connect Path Planning";
    const string window_name4 = "GRRT-Connect Path Planning";
    //设置地图的宽和高
    const int width = 1000;
    const int height = 1000;
    const float stride = 10;        //扩展树的步长
    const float threashold = 0.05;  //分界概率值
    const int MaxIteration = 20000; //迭代上限
    const int rand_precition = 5;   //坐标精确位数
    cv::Point2f start(0, 0), end(900, 900);
    float radius = 40;
    float grow_rate = 1.5;
    const float dismin = 50;

    for (int i = 1; i < 60; ++i) {
        compare_planner2(window_name1, window_name2, window_name3, window_name4, width, height,
                         stride, threashold, MaxIteration, rand_precition, radius, grow_rate,
                         dismin, start, end, i);
    }
    for (int i = 0; i < rrtTime.size(); ++i) {
        rrtRecord << to_string(rrtTime[i]) + ",";
        rrtStarRecord << to_string(rrtStarTime[i]) + ",";
        rrtConnectRecord << to_string(rrtConnectTime[i]) + ",";
        GrrtConnectRecord << to_string(GrrtConnectTime[i]) + ",";
    }
    rrtRecord << "\n";
    rrtStarRecord << "\n";
    rrtConnectRecord << "\n";
    GrrtConnectRecord << "\n";
    for (int i = 0; i < rrtTime.size(); ++i) {
        rrtRecord << to_string(rrtLength[i]) + ",";
        rrtStarRecord << to_string(rrtStarLength[i]) + ",";
        rrtConnectRecord << to_string(rrtConnectLength[i]) + ",";
        GrrtConnectRecord << to_string(GrrtConnectLength[i]) + ",";
    }
    rrtRecord.close();
    rrtStarRecord.close();
    rrtConnectRecord.close();
    GrrtConnectRecord.close();
    // cv::Mat planMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    // //创建地图
    // Map map(width, height, planMap, window_name4);
    // //地图初始化
    // map.init();

    // //绘制障碍物
    // map.drawObstacles();
    // RRT算法
    // RRT rrt(start, end, stride, threashold, MaxIteration, rand_precition, &map);
    // rrt.init();
    // rrt.planning();

    // RRT*算法
    // RRT_STAR rrt_star(start, end, stride, threashold, MaxIteration, rand_precition,
    // &map,
    //                   radius);
    // rrt_star.init();
    // rrt_star.planning();

    // RRT_Connect算法
    // RRT_Connect rrt_connect(start, end, stride, threashold, MaxIteration, rand_precition, &map);
    // rrt_connect.init();
    // rrt_connect.planning();

    // GRRT_Connect算法
    // GRRT_Connect Grrt_connect(start, end, stride, threashold, MaxIteration, rand_precition, &map,
    //                           grow_rate, dismin);
    // Grrt_connect.init();
    // Grrt_connect.planning();
    // cv::waitKey(0);
}
