#include "ceres_solver.hpp"

namespace solver_plugins
{

/*****************************************************************************/
CeresSolver::CeresSolver()
: problem_(NULL),
  was_constant_set_(false),
  nodes_(new std::unordered_map<int, Eigen::Vector3d>()),
  blocks_(new std::unordered_map<std::size_t, ceres::ResidualBlockId>())
/*****************************************************************************/
{
}

/*****************************************************************************/
void CeresSolver::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
/*****************************************************************************/
{
        logger_ = node->get_logger();

        std::string solver_type, preconditioner_type, dogleg_type,
            trust_strategy, loss_fn, mode;
        if (!node->has_parameter("ceres_linear_solver"))
        {
                node->declare_parameter(
                    "ceres_linear_solver",
                    rclcpp::ParameterValue(std::string("SPARSE_NORMAL_CHOLESKY")));
        }
        solver_type = node->get_parameter("ceres_linear_solver").as_string();

        if (!node->has_parameter("ceres_preconditioner"))
        {
                node->declare_parameter(
                    "ceres_preconditioner",
                    rclcpp::ParameterValue(std::string("JACOBI")));
        }
        preconditioner_type = node->get_parameter("ceres_preconditioner").as_string();

        if (!node->has_parameter("ceres_dogleg_type"))
        {
                node->declare_parameter(
                    "ceres_dogleg_type",
                    rclcpp::ParameterValue(std::string("TRADITIONAL_DOGLEG")));
        }
        dogleg_type = node->get_parameter("ceres_dogleg_type").as_string();

        if (!node->has_parameter("ceres_trust_strategy"))
        {
                node->declare_parameter(
                    "ceres_trust_strategy",
                    rclcpp::ParameterValue(std::string("LM")));
        }
        trust_strategy = node->get_parameter("ceres_trust_strategy").as_string();

        if (!node->has_parameter("ceres_loss_function"))
        {
                node->declare_parameter(
                    "ceres_loss_function",
                    rclcpp::ParameterValue(std::string("None")));
        }
        loss_fn = node->get_parameter("ceres_loss_function").as_string();


        if (!node->has_parameter("mode"))
        {
                node->declare_parameter(
                    "mode",
                    rclcpp::ParameterValue(std::string("mapping")));
        }
        mode = node->get_parameter("mode").as_string();

        // debug_logging_ = node->get_parameter("debug_logging").as_bool();
        debug_logging_ = true;

        corrections_.clear();
        first_node_ = nodes_->end();

        // formulate problem
        angle_manifold_ = AngleManifold::Create();

        // choose loss function default squared loss (NULL)
        loss_function_ = NULL;
        if (loss_fn == "HuberLoss")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using HuberLoss loss function.");
                loss_function_ = new ceres::HuberLoss(0.7);
        }
        else if (loss_fn == "CauchyLoss")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using CauchyLoss loss function.");
                loss_function_ = new ceres::CauchyLoss(0.7);
        }

        // choose linear solver default CHOL
        options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

        if (solver_type == "SPARSE_SCHUR")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using SPARSE_SCHUR solver.");
                options_.linear_solver_type = ceres::SPARSE_SCHUR;
        }
        else if (solver_type == "ITERATIVE_SCHUR")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using ITERATIVE_SCHUR solver.");
                options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
        }
        else if (solver_type == "CGNR")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using CGNR solver.");
                options_.linear_solver_type = ceres::CGNR;
        }

        // choose preconditioner default Jacobi
        options_.preconditioner_type = ceres::JACOBI;
        if (preconditioner_type == "IDENTITY")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using IDENTITY preconditioner.");
                options_.preconditioner_type = ceres::IDENTITY;
        }
        else if (preconditioner_type == "SCHUR_JACOBI")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using SCHUR_JACOBI preconditioner.");
                options_.preconditioner_type = ceres::SCHUR_JACOBI;
        }

        if (options_.preconditioner_type == ceres::CLUSTER_JACOBI ||
            options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
        {
                // default canonical view is O(n^2) which is unacceptable for
                // problems of this size
                options_.visibility_clustering_type = ceres::SINGLE_LINKAGE;
        }

        // choose trust region strategy default LM
        options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        if (trust_strategy == "DOGLEG")
        {
                RCLCPP_INFO(
                    node->get_logger(),
                    "CeresSolver: Using DOGLEG trust region strategy.");
                options_.trust_region_strategy_type = ceres::DOGLEG;
        }

        // choose dogleg type default traditional
        if (options_.trust_region_strategy_type == ceres::DOGLEG)
        {
                options_.dogleg_type = ceres::TRADITIONAL_DOGLEG;
                if (dogleg_type == "SUBSPACE_DOGLEG")
                {
                        RCLCPP_INFO(
                            node->get_logger(),
                            "CeresSolver: Using SUBSPACE_DOGLEG dogleg type.");
                        options_.dogleg_type = ceres::SUBSPACE_DOGLEG;
                }
        }

        // a typical ros map is 5cm, this is 0.001, 50x the resolution
        options_.function_tolerance = 1e-3;
        options_.gradient_tolerance = 1e-6;
        options_.parameter_tolerance = 1e-3;

        options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options_.max_num_consecutive_invalid_steps = 3;
        options_.max_consecutive_nonmonotonic_steps =
            options_.max_num_consecutive_invalid_steps;
        options_.num_threads = 50;
        options_.use_nonmonotonic_steps = true;
        options_.jacobi_scaling = true;

        options_.min_relative_decrease = 1e-3;

        options_.initial_trust_region_radius = 1e4;
        options_.max_trust_region_radius = 1e8;
        options_.min_trust_region_radius = 1e-16;

        options_.min_lm_diagonal = 1e-6;
        options_.max_lm_diagonal = 1e32;

        if (options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY)
        {
                options_.dynamic_sparsity = true;
        }

        if (mode == std::string("localization"))
        {
                // doubles the memory footprint, but lets us remove contraints faster
                options_problem_.enable_fast_removal = true;
        }

        // we do not want the problem definition to own these objects, otherwise they get
        // deleted along with the problem
        options_problem_.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

        problem_ = new ceres::Problem(options_problem_);
}

/*****************************************************************************/
void CeresSolver::compute()
/*****************************************************************************/
{
        boost::mutex::scoped_lock lock(nodes_mutex_);

        if (nodes_->size() == 0) {
                RCLCPP_WARN(
                    logger_,
                    "CeresSolver: Ceres was called when there are no nodes."
                    " This shouldn't happen.");
                return;
        }

        // populate contraint for static initial pose
        if (!was_constant_set_ && first_node_ != nodes_->end() &&
            problem_->HasParameterBlock(&first_node_->second(0)) &&
            problem_->HasParameterBlock(&first_node_->second(1)) &&
            problem_->HasParameterBlock(&first_node_->second(2)))
        {
                RCLCPP_DEBUG(
                    logger_,
                    "CeresSolver: Setting first node as a constant pose:"
                    "%0.2f, %0.2f, %0.2f.",
                    first_node_->second(0),
                    first_node_->second(1), first_node_->second(2));
                problem_->SetParameterBlockConstant(&first_node_->second(0));
                problem_->SetParameterBlockConstant(&first_node_->second(1));
                problem_->SetParameterBlockConstant(&first_node_->second(2));
                was_constant_set_ = !was_constant_set_;
        }

        ceres::Solver::Summary summary;
        ceres::Solve(options_, problem_, &summary);
        if (debug_logging_) {
                std::cout << summary.FullReport() << '\n';
        }

        if (!summary.IsSolutionUsable()) {
                RCLCPP_WARN(
                    logger_, "CeresSolver: "
                             "Ceres could not find a usable solution to optimize.");
                return;
        }

        // store corrected poses
        if (!corrections_.empty()) {
                corrections_.clear();
        }
        corrections_.reserve(nodes_->size());
        karto::Pose2 pose;
        for (ConstGraphIterator iter = nodes_->begin(); iter != nodes_->end(); ++iter)
        {
                pose.setX(iter->second(0));
                pose.setY(iter->second(1));
                pose.setHeading(iter->second(2));
                corrections_.push_back(std::make_pair(iter->first, pose));
        }
}

/*****************************************************************************/
void CeresSolver::addNode(karto::Vertex<karto::LocalizedRangeScan> *vertex)
/*****************************************************************************/
{
        // store nodes
        if (!vertex) {
                return;
        }

        karto::Pose2 pose = vertex->getObject()->getCorrectedPose();
        Eigen::Vector3d pose2d(pose.getX(), pose.getY(), pose.getHeading());

        const int id = vertex->getObject()->getScanId();

        boost::mutex::scoped_lock lock(nodes_mutex_);
        nodes_->insert(std::pair<int, Eigen::Vector3d>(id, pose2d));

        if (nodes_->size() == 1) {
                first_node_ = nodes_->find(id);
        }
}

/*****************************************************************************/
void CeresSolver::addConstraint(karto::Edge<karto::LocalizedRangeScan> *edge)
/*****************************************************************************/
{
        // get IDs in graph for this edge
        boost::mutex::scoped_lock lock(nodes_mutex_);

        if (!edge) {
                return;
        }

        const int node1 = edge->getSource()->getObject()->getScanId();
        GraphIterator node1it = nodes_->find(node1);
        const int node2 = edge->getTarget()->getObject()->getScanId();
        GraphIterator node2it = nodes_->find(node2);

        if (node1it == nodes_->end() ||
            node2it == nodes_->end() || 
            node1it == node2it) {
                RCLCPP_WARN(
                    logger_,
                    "CeresSolver: Failed to add constraint, could not find nodes.");
                return;
        }

        // extract transformation
        karto::LinkInfo *link_info = (karto::LinkInfo *)(edge->getLabel());
        karto::Pose2 diff = link_info->getPoseDifference();
        Eigen::Vector3d pose2d(diff.getX(), diff.getY(), diff.getHeading());

        karto::Matrix3 precision_matrix = link_info->getCovariance().inverse();
        Eigen::Matrix3d information;
        information(0, 0) = precision_matrix(0, 0);
        information(0, 1) = information(1, 0) = precision_matrix(0, 1);
        information(0, 2) = information(2, 0) = precision_matrix(0, 2);
        information(1, 1) = precision_matrix(1, 1);
        information(1, 2) = information(2, 1) = precision_matrix(1, 2);
        information(2, 2) = precision_matrix(2, 2);
        Eigen::Matrix3d sqrt_information = information.llt().matrixU();

        // populate residual and parameterization for heading normalization
        ceres::CostFunction *cost_function = PoseGraph2dErrorTerm::Create(pose2d(0),
                                                                          pose2d(1), pose2d(2), sqrt_information);
        ceres::ResidualBlockId block = problem_->AddResidualBlock(
            cost_function, loss_function_,
            &node1it->second(0), &node1it->second(1), &node1it->second(2),
            &node2it->second(0), &node2it->second(1), &node2it->second(2));
        problem_->SetManifold(&node1it->second(2),
                              angle_manifold_);
        problem_->SetManifold(&node2it->second(2),
                              angle_manifold_);

        blocks_->insert(std::pair<std::size_t, ceres::ResidualBlockId>(
                GetHash(node1, node2), block));
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector &CeresSolver::getCorrections() const
/*****************************************************************************/
{
        return corrections_;
}

/*****************************************************************************/
std::unordered_map<int, Eigen::Vector3d> *CeresSolver::getGraph()
/*****************************************************************************/
{
        boost::mutex::scoped_lock lock(nodes_mutex_);
        return nodes_;
}

} // namespace solver_plugins