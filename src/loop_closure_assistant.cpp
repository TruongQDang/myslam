#include "myslam/loop_closure_assistant.hpp"

namespace loop_closure_assistant
{

/*****************************************************************************/
template<class NodeT>
LoopClosureAssistant::LoopClosureAssistant(
        NodeT node,
        mapper_utils::Mapper * mapper,
        mapper_utils::ScanManager* scan_holder)
: mapper_(mapper), scan_holder_(scan_holder),
  clock_(node->get_clock()), logger_(node->get_logger()),
  parameters_interface_(node->get_node_parameters_interface())
/*****************************************************************************/
{
        solver_ = mapper_->getScanSolver();

        marker_publisher_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
            "myslam/graph_visualization", rclcpp::QoS(1));
            
        // map_frame_ = node->get_parameter("map_frame").as_string();
        map_frame_ = std::string("map");
}

template LoopClosureAssistant::LoopClosureAssistant<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode>,
        mapper_utils::Mapper *,
        mapper_utils::ScanManager *);

/*****************************************************************************/
void LoopClosureAssistant::setMapper(mapper_utils::Mapper *mapper)
/*****************************************************************************/
{
        mapper_ = mapper;
}

/*****************************************************************************/
void LoopClosureAssistant::publishGraph()
/*****************************************************************************/
{
        auto graph = solver_->getGraph();

        if (graph->size() == 0) {
                return;
        }

        // RCLCPP_DEBUG(logger_, "Graph size: %zu", graph->size());
        RCLCPP_INFO(logger_, "Graph size: %zu", graph->size());

        const auto &vertices = mapper_->getGraph()->getVertices();
        const auto &edges = mapper_->getGraph()->getEdges();

        visualization_msgs::msg::MarkerArray marray;

        visualization_msgs::msg::Marker m = vis_utils::toMarker(map_frame_, "myslam", 0.1, clock_);

        // add graph nodes
        for (const auto &vertex : vertices) {
                const auto &pose = vertex.second->getObject()->getCorrectedPose();
                m.id = vertex.first;
                m.pose.position.x = pose.getX();
                m.pose.position.y = pose.getY();
                marray.markers.push_back(m);
        }

        // add graph edges
        visualization_msgs::msg::Marker edges_marker;
        edges_marker.header.frame_id = map_frame_;
        edges_marker.header.stamp = clock_->now();
        edges_marker.id = 0;
        edges_marker.ns = "myslam_edges";
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.pose.orientation.w = 1;
        edges_marker.scale.x = 0.05;
        edges_marker.color.b = 1;
        edges_marker.color.a = 1;
        edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
        edges_marker.points.reserve(edges.size() * 2);

        for (const auto &edge : edges) {
                int source_id = edge->getSource()->getObject()->getScanId();
                const auto &pose0 = edge->getSource()->getObject()->getCorrectedPose();
                geometry_msgs::msg::Point p0;
                p0.x = pose0.getX();
                p0.y = pose0.getY();

                int target_id = edge->getTarget()->getObject()->getScanId();
                const auto &pose1 = edge->getTarget()->getObject()->getCorrectedPose();
                geometry_msgs::msg::Point p1;
                p1.x = pose1.getX();
                p1.y = pose1.getY();

                edges_marker.points.push_back(p0);
                edges_marker.points.push_back(p1);
        }

        marray.markers.push_back(edges_marker);

        // if disabled, clears out old markers
        marker_publisher_->publish(marray);
}

} // loop_closure_assistant