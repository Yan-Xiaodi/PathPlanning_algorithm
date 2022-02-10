#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <opencv2/core/types.hpp>
#include <vector>

class Obstacle
{
  public:
    Obstacle() {}
    Obstacle(std::vector<cv::Point2f> v) : vertices(v) {}
    ~Obstacle() {}

    bool empty() { return vertices.empty(); }
    void clear() { vertices.clear(); }

    void push(cv::Point2f point) { vertices.push_back(point); }
    void push(float x, float y) { vertices.push_back({x, y}); }

    std::vector<cv::Point2f> getVertices() { return vertices; }
    size_t getPointsSize() { return vertices.size(); }
    std::vector<cv::Point2f>& setVertices() { return vertices; }

  private:
    std::vector<cv::Point2f> vertices;
};

#endif