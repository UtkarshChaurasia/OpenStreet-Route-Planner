#include "route_planner.h"
#include <algorithm>
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
  // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
  // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  RoutePlanner::start_node = &RoutePlanner::m_Model.FindClosestNode(start_x, start_y);
  RoutePlanner::end_node = &RoutePlanner::m_Model.FindClosestNode(end_x, end_y);
}

// Implementing the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}




// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// - Using the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Using CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    // Populate the neighbors of the current node
    current_node->FindNeighbors();
    
    // For each neighbor
    for (auto nd : current_node->neighbors) {
        
        // Set parent to current
        nd->parent = current_node;
        
         // Set neighbor's h by the appropriate function
        nd->h_value = CalculateHValue(nd);
        
        // Set neighbor's g to current's g + distance between current node and neighbor
        nd->g_value = current_node->distance(*nd) + current_node->g_value;
        
        // Add neighbor to open_list
        open_list.push_back(nd);
        
        // Set neighbor's visited attribute to true
        nd->visited = true;
    }
}


// NextNode method to sort the open list and return the next node.
// - Sorting the open_list according to the sum of the h value and g value.
// - Creating a pointer to the node in the list with the lowest sum.
// - Removing that node from the open_list.
// - Return the pointer.
RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *ndnext;
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *nd1, const RouteModel::Node *nd2) {
        return (nd1->h_value + nd1->g_value) > (nd2->h_value + nd2->g_value);
        });
    ndnext = open_list.back();
    open_list.pop_back();
    return ndnext;
}



// ConstructFinalPath method to return the final path found from your A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *nd;
    nd = current_node;
    path_found.push_back(*nd);
    
    // For each node in the path
    while(nd != start_node) {
        
        // Store node in the path
        path_found.push_back(*(nd->parent));
        
        // Add the distance
        distance += nd->distance(*(nd->parent));
        
        // Move to the parent node
        nd = nd->parent;
    }
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}



// Implementing A* Search algorithm here.
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
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
