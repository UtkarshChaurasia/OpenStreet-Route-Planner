#include "route_planner.h"
#include <algorithm>
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
  RoutePlanner::start_node = &RoutePlanner::m_Model.FindClosestNode(start_x, start_y);
  RoutePlanner::end_node = &RoutePlanner::m_Model.FindClosestNode(end_x, end_y);
}
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto nd : current_node->neighbors) {
        nd->parent = current_node;
        nd->h_value = CalculateHValue(nd);
        nd->g_value = current_node->distance(*nd) + current_node->g_value;
        open_list.push_back(nd);
        nd->visited = true;
    }
}
RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *ndnext;
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *nd1, const RouteModel::Node *nd2) {
        return (nd1->h_value + nd1->g_value) > (nd2->h_value + nd2->g_value);
        });
    ndnext = open_list.back();
    open_list.pop_back();
    return ndnext;
}
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *nd;
    nd = current_node;
    path_found.push_back(*nd);
    while(nd != start_node) {
        path_found.push_back(*(nd->parent));
        distance += nd->distance(*(nd->parent));
        nd = nd->parent;
    }
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
    open_list.push_back(current_node);
    current_node->visited=true;
    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
}
