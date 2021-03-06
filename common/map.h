#ifndef MAP_H
#define MAP_H

/**地图类
 * 添加、删除障碍物
 * 绘制节点、路径
 */
#include "obstacle.h"
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>
#include <thread>
#include <unistd.h>
#include <vector>

class Map
{
  public:
    Map(int width, int height, cv::Mat& map_, const std::string name)
        : map_x(width), map_y(height), planMap(map_), window_name(name), ImageBuffer(1, planMap)
    {
        video_frame = planMap;
        writed = 0;
        thread_ = new pthread_t;
    }

    Map(const Map& rhs)
    {
        writer = rhs.writer;
        writed = rhs.writed;
        record = rhs.record;
        obstacle_list = rhs.obstacle_list;
        planMap = rhs.planMap;
        window_name = rhs.window_name;
        map_x = rhs.map_x;
        map_y = rhs.map_y;
        thread_ = rhs.thread_;
        ImageBuffer = rhs.ImageBuffer;
        WriteDown = rhs.WriteDown;
    }

    void init();

    void recordToVideo();

    void addObstacle(Obstacle);

    //鼠标事件：将鼠标点击的位置作为一个顶点加入到障碍物中
    //注意：障碍物形成凸体的方式和鼠标点击的顺序相关，按顺时针/逆时针逐个点击顶点；
    static void onMouseHandle(int event, int x, int y, int flags, void* param);

    //绘制顶点
    void drawObjPoints();

    void drawPoint(cv::Point2f&);

    void drawPointColor(cv::Point2f&, cv::Scalar);

    std::vector<cv::Point> Point2fToPoint2i(const std::vector<cv::Point2f>&);

    void drawAllObstacles();

    void drawObstacle();

    void drawObstacles();

    void drawLine(cv::Point2f, cv::Point2f);

    void drawLineColor(cv::Point2f, cv::Point2f, cv::Scalar);

    void drawText(const std::string&, cv::Point);

    int getWidth();

    int getHeight();

    std::vector<Obstacle> getObstacleList();

    cv::Mat getPlanMap();

    const std::string getWindowName();

    void show();

    static void* WriteVideo(void*);

    void closeVideo();

    ~Map();

  public:
    cv::VideoWriter writer;
    int writed;
    bool record = false; //是否将路径搜索过程记录为视频

  private:
    std::vector<Obstacle> obstacle_list; //障碍物集合
    static Obstacle obj;
    cv::Mat planMap;
    cv::Mat video_frame;
    std::string window_name;
    int map_x;                        //地图的长(x)
    int map_y;                        //地图的高(y)
    pthread_t* thread_;               //线程
    std::mutex locker;                //互斥锁
    std::vector<cv::Mat> ImageBuffer; //图像缓存
    bool WriteDown = false;           //视频是否完全写入
};
#endif
