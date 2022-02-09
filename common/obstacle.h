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

    void push(cv::Point2f point) { vertices.push_back(point); }
    void push(float x, float y) { vertices.push_back({x, y}); }
    void clear() { vertices.clear(); }
    std::vector<cv::Point2f> getVertices() { return vertices; }
    bool empty() { return vertices.empty(); }
    size_t getPointsSize() { return vertices.size(); }

  private:
    std::vector<cv::Point2f> vertices;
};

#endif