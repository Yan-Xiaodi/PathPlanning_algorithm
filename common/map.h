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

    void init();

    void addObstacle(Obstacle);

    //鼠标事件：将鼠标点击的位置作为一个顶点加入到障碍物中
    //注意：障碍物形成凸体的方式和鼠标点击的顺序相关，按顺时针/逆时针逐个点击顶点；
    static void onMouseHandle(int event, int x, int y, int flags, void* param);

    //绘制顶点
    void drawObjPoints();

    void drawPoint(cv::Point2f&);

    std::vector<cv::Point> Point2fToPoint2i(const std::vector<cv::Point2f>&);

    void drawAllObstacles();

    void drawObstacle();

    void drawObstacles();

    void drawLine(cv::Point2f, cv::Point2f);
    void drawLine(cv::Point2f, cv::Point2f, cv::Scalar);

    void drawText(const std::string&, cv::Point);

    int getWidth();

    int getHeight();

    std::vector<Obstacle> getObstacleList();

    cv::Mat getPlanMap();

    const std::string getWindowName();

  private:
    std::vector<Obstacle> obstacle_list; //障碍物集合
    static Obstacle obj;
    cv::Mat planMap;
    const std::string window_name;
    int map_x; //地图的长(x)
    int map_y; //地图的高(y)
};
#endif
