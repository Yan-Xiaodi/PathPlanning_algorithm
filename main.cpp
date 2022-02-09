#include "map.h"
#include "rrt.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

Obstacle obj;
//鼠标事件：将鼠标点击的位置作为一个顶点加入到障碍物中
//注意：障碍物形成凸体的方式和鼠标点击的顺序相关，按顺时针/逆时针逐个点击顶点；
static void onMouseHandle(int event, int x, int y, int flags, void* param)
{

    if (event == CV_EVENT_LBUTTONDOWN) {
        // cv::Mat map = *((cv::Mat*)param);
        obj.push((float)x, (float)y);
        // printf("%d %d\n", x, y);
    }
}

//绘制顶点
void drawPoints(cv::Mat& planMap, Obstacle& obj)
{
    for (int i = 0; i < obj.getPointsSize(); ++i) {
        cv::Point p(obj.getVertices()[i].x, obj.getVertices()[i].y);
        cv::circle(planMap, p, 2, cv::Scalar(0), -1);
    }
}

vector<cv::Point> Point2fToPoint2i(const vector<cv::Point2f>& points2f)
{
    vector<cv::Point> points2i;
    for (int i = 0; i < points2f.size(); ++i) {
        points2i.push_back(cv::Point(points2f[i].x, points2f[i].y));
    }
    return points2i;
}

void drawObstacle(cv::Mat& planMap, Obstacle& obj, int index)
{
    //填充obstacle
    vector<vector<cv::Point>> hull{Point2fToPoint2i(obj.getVertices())};
    cv::drawContours(planMap, hull, 0, cv::Scalar(0), -1);
}

void drawObstacle(cv::Mat& planMap, const string& window_name,
                  vector<Obstacle>& obstacle_list)
{
    // cv::setMouseCallback(window_name, onMouseHandle, (void*)&planMap);
    while (1) {
        cv::imshow(window_name, planMap);
        drawPoints(planMap, obj);
        if (cv::waitKey(50) == 32) {
            //按下空格说明一个障碍物绘制完成，将其加入到obstacle_list当中
            //如果按下空格后，obj为空，说明所有障碍物绘制完毕，退出循环
            if (obj.empty())
                break;
            obstacle_list.push_back(obj);
            drawObstacle(planMap, obj, obstacle_list.size());
            obj.clear();
        }
    }
}

int main(int argc, char** argv)
{
    const string window_name = "RRT Path Planning";
    //设置地图的宽和高
    const int width = 1000;
    const int height = 1000;
    //创建画布
    cv::Mat planMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //创建地图
    Map map(width, height, planMap, window_name);
    //地图初始化
    map.init();
    //绘制障碍物
    map.drawObstacles();

    // RRT算法初始化
    cv::Point2f start(0, 0), end(999, 999);

    // RRT rrt(start, end, 20, 0.4, 20000, 3, obstacle_list);
    // rrt.init(width, height);
    // cv::Point2f pt(20, 20);
    // auto point = rrt.getSamplePoint();
    // cout << point.x << " " << point.y << endl;
}
