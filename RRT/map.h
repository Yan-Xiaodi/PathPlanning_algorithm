#ifndef MAP_H
#define MAP_H

/**地图类
 * 添加、删除障碍物
 * 绘制节点、路径
 */
#include "obstacle.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
class Map
{
  public:
    Map(int width, int height, cv::Mat& map_, const std::string name)
        : map_x(width), map_y(height), planMap(map_), window_name(name)
    {
    }

    void init()
    {
        cv::setMouseCallback(window_name, onMouseHandle, 0);
        cv::namedWindow(window_name);
    }

    //鼠标事件：将鼠标点击的位置作为一个顶点加入到障碍物中
    //注意：障碍物形成凸体的方式和鼠标点击的顺序相关，按顺时针/逆时针逐个点击顶点；
    static void onMouseHandle(int event, int x, int y, int flags, void* param)
    {

        if (event == CV_EVENT_LBUTTONDOWN) {
            obj.push((float)x, (float)y);
        }
    }

    //绘制顶点
    void drawObjPoints()
    {
        for (int i = 0; i < obj.getPointsSize(); ++i) {
            cv::Point p(obj.getVertices()[i].x, obj.getVertices()[i].y);
            cv::circle(planMap, p, 2, cv::Scalar(0), -1);
        }
    }

    void drawPoints(cv::Point2f& point)
    {
        cv::circle(planMap, cv::Point(point.x, point.y), 2, cv::Scalar(0, 150, 0), -1);
    }

    std::vector<cv::Point> Point2fToPoint2i(const std::vector<cv::Point2f>& points2f)
    {
        std::vector<cv::Point> points2i;
        for (int i = 0; i < points2f.size(); ++i) {
            points2i.push_back(cv::Point(points2f[i].x, points2f[i].y));
        }
        return points2i;
    }

    void drawObstacle()
    {
        //填充obstacle
        std::vector<std::vector<cv::Point>> hull{Point2fToPoint2i(obj.getVertices())};
        cv::drawContours(planMap, hull, 0, cv::Scalar(0), -1);
    }

    void drawObstacles()
    {
        // cv::setMouseCallback(window_name, onMouseHandle, (void*)&planMap);
        while (1) {
            cv::imshow(window_name, planMap);
            drawObjPoints();
            if (cv::waitKey(50) == 32) {
                //按下空格说明一个障碍物绘制完成，将其加入到obstacle_list当中
                //如果按下空格后，obj为空，说明所有障碍物绘制完毕，退出循环
                if (obj.empty())
                    break;
                obstacle_list.push_back(obj);
                drawObstacle();
                obj.clear();
            }
        }
    }

    void drawLine(cv::Mat&) {}

  private:
    std::vector<Obstacle> obstacle_list; //障碍物集合
    static Obstacle obj;
    cv::Mat planMap;
    const std::string window_name;
    int map_x; //地图的长(x)
    int map_y; //地图的高(y)
};
Obstacle Map::obj;

#endif