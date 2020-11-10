#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  float dist = node->distance(*this->end_node);
  return dist;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }
}

bool RoutePlanner::CompareNodes(RouteModel::Node *a, RouteModel::Node *b)
{
    float aVal = a->g_value + a->h_value;
    float bVal = b->g_value + b->h_value;
    return aVal > bVal;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), CompareNodes);
    RouteModel::Node *last_element = this->open_list.back();
    this->open_list.pop_back();
    return last_element; 
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // NOTE: Will need to reverse the oder of path_found vector.
    RouteModel::Node *current_ptr = current_node;
    while (current_ptr->parent !=nullptr)
    {
        distance += current_ptr->distance(*current_ptr->parent);
        path_found.push_back(*current_ptr);
        current_ptr = current_ptr->parent;
    }
  	path_found.push_back(*this->start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
	
    // TODO: Implement your solution here.
    this->open_list.push_back(this->start_node);
    this->start_node->visited = true;
    while (this->open_list.size() > 0 && current_node != this->end_node)
    {
        current_node = NextNode();
        AddNeighbors(current_node);
    }
  	m_Model.path = ConstructFinalPath(this->end_node);
}
