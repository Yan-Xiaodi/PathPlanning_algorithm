#include "map.h"

Obstacle Map::obj;
void Map::init()
{
    cv::namedWindow(window_name);
    cv::setMouseCallback(window_name, onMouseHandle, 0);
}

//鼠标事件：将鼠标点击的位置作为一个顶点加入到障碍物中
//注意：障碍物形成凸体的方式和鼠标点击的顺序相关，按顺时针/逆时针逐个点击顶点；
void Map::onMouseHandle(int event, int x, int y, int flags, void* param)
{

    if (event == CV_EVENT_LBUTTONDOWN) {
        obj.push((float)x, (float)y);
    }
}

//绘制顶点
void Map::drawObjPoints()
{
    for (int i = 0; i < obj.getPointsSize(); ++i) {
        cv::Point p(obj.getVertices()[i].x, obj.getVertices()[i].y);
        cv::circle(planMap, p, 2, cv::Scalar(0), -1);
    }
}

void Map::drawPoint(cv::Point2f& point)
{
    cv::circle(planMap, cv::Point(point.x, point.y), 3, cv::Scalar(0, 0, 0), -1);
}

std::vector<cv::Point> Map::Point2fToPoint2i(const std::vector<cv::Point2f>& points2f)
{
    std::vector<cv::Point> points2i;
    for (int i = 0; i < points2f.size(); ++i) {
        points2i.push_back(cv::Point(points2f[i].x, points2f[i].y));
    }
    return points2i;
}

void Map::drawObstacle()
{
    //填充obstacle
    std::vector<std::vector<cv::Point>> hull{Point2fToPoint2i(obj.getVertices())};
    cv::drawContours(planMap, hull, 0, cv::Scalar(0), -1);
}

void Map::drawObstacles()
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

void Map::drawLine(cv::Point2f p1, cv::Point2f p2)
{
    cv::line(planMap, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(0, 150, 0),
             2);
}

void Map::drawText(const std::string& text, cv::Point org = cv::Point(20, 20))
{
    cv::putText(planMap, text, org, cv::FONT_ITALIC, 0.5, cv::Scalar(0));
}

int Map::getWidth() { return map_x; }

int Map::getHeight() { return map_y; }

std::vector<Obstacle> Map::getObstacleList() { return obstacle_list; }

cv::Mat Map::getPlanMap() { return planMap; }

const std::string Map::getWindowName() { return window_name; }