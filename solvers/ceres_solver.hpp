#ifndef CERES_SOLVER_HPP_
#define CERES_SOLVER_HPP_

#include "karto_sdk/myslam_types.hpp"
#include "karto_sdk/mapper_utils.hpp"
#include "ceres_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace solver_plugins
{

using namespace ::myslam_types;

class CeresSolver : public mapper_utils::ScanSolver
{
private:
        // karto
        mapper_utils::ScanSolver::IdPoseVector corrections_;

        // ceres
        ceres::Solver::Options options_;
        ceres::Problem::Options options_problem_;
        ceres::LossFunction *loss_function_;
        ceres::Problem *problem_;
        ceres::Manifold *angle_manifold_;
        bool was_constant_set_, debug_logging_;

        // graph
        std::unordered_map<int, Eigen::Vector3d> *nodes_;
        std::unordered_map<size_t, ceres::ResidualBlockId> *blocks_;
        std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
        boost::mutex nodes_mutex_;

        // ros
        rclcpp::Logger logger_{rclcpp::get_logger("CeresSolver")};

public:
        CeresSolver();

        virtual void compute(); // Solve
        virtual void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        // Adds a node to the solver
        virtual void addNode(mapper_utils::Vertex<mapper_utils::LocalizedRangeScan> *pVertex);
        // Adds a constraint to the solver
        virtual void addConstraint(mapper_utils::Edge<mapper_utils::LocalizedRangeScan> *pEdge);

        std::unordered_map<int, Eigen::Vector3d> *getGraph();

        // Get corrected poses after optimization
        virtual const mapper_utils::ScanSolver::IdPoseVector &getCorrections() const;
}; // CeresSolver

} // namespace solver_plugins

#endif // CERES_SOLVER_HPP_