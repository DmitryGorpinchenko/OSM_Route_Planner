#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model)
    , start_node(&model.FindClosestNode(start_x * 0.01f, start_y * 0.01f))
    , end_node(&model.FindClosestNode(end_x * 0.01f, end_y * 0.01f))
{}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *node) {
    node->FindNeighbors();
    for (auto *n : node->neighbors) {
        const double g = node->g_value + n->distance(*node);
        if ((!n->visited) || (g < n->g_value)) {
            n->parent = node;
            n->g_value = g;
            n->h_value = CalculateHValue(n);
            n->visited = true;
            open_set.insert(n);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    if (open_set.empty()) {
        return nullptr;
    }

    auto it = std::min_element(open_set.begin(), open_set.end(), [](auto *l, auto *r) { return l->g_value + l->h_value < r->g_value + r->h_value; });
    
    auto *res = *it;
    open_set.erase(it);
    return res;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node const *node) {
    distance = node->g_value * m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    std::vector<RouteModel::Node> path_found;
    do {
        path_found.push_back(*node);
        node = node->parent;
    } while (node);
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}


void RoutePlanner::AStarSearch() {
    start_node->g_value = 0.0;
    start_node->h_value = CalculateHValue(start_node);
    start_node->visited = true;

    for (auto *node = start_node; node; node = NextNode()) {
        if (node == end_node) {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }
        AddNeighbors(node);
    }
}