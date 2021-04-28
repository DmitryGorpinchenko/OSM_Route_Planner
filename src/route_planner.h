#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <unordered_set>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);

    float GetDistance() const {return distance;}
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node const *node);
    RouteModel::Node *NextNode();

  private:
    RouteModel &m_Model;
  
    std::unordered_set<RouteModel::Node *> open_set;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    float distance = 0.0f;
};

#endif