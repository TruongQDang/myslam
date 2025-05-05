#ifndef KARTO_SDK_MAPPER_HPP
#define KARTO_SDK_MAPPER_HPP

#include <iostream>
#include <unordered_map>
#include <queue>
#include <set>

#include "tbb/parallel_for_each.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "math.hpp"
#include "karto.hpp"


namespace karto
{

typedef std::vector<LocalizedRangeScan *> LocalizedRangeScanVector;
typedef std::map<int, LocalizedRangeScan *> LocalizedRangeScanMap;
typedef std::vector<Eigen::Vector2d> PointVectorDouble;

//////////////////////////////////////////////////////////////
template <typename T>
class Vertex;
class LinkInfo;

template <typename T>
class Edge
{
private:
        Vertex<T> *source_;
        Vertex<T> *target_;
        LinkInfo *label_;

public:
        Edge(Vertex<T> *source, Vertex<T> *target)
            : source_(source),
              target_(target),
              label_(nullptr)
        {
                source_->addEdge(this);
                target_->addEdge(this);
        }

public:
        /**
         * Gets the target vertex
         * @return target vertex
         */
        inline Vertex<T> *getTarget() const
        {
                return target_;
        }

        /**
         * Sets the link payload
         * @param pLabel
         */
        inline void setLabel(LinkInfo *label)
        {
                label_ = label;
        }

        /**
         * Gets the link info
         * @return link info
         */
        inline LinkInfo *getLabel()
        {
                return label_;
        }

        /**
         * Gets the source vertex
         * @return source vertex
         */
        inline Vertex<T> *getSource() const
        {
                return source_;
        }

}; // Edge

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

template <typename T>
class Vertex
{
        friend class Edge<T>;

private:
        T *object_;
        std::vector<Edge<T> *> edges_;
        double score_;

public:
        explicit Vertex(T *object)
            : object_(object), score_(1.0)
        {
        }

public:
        /**
         * Gets the object associated with this vertex
         * @return the object
         */
        inline T *getObject() const
        {
                return object_;
        }

        /**
         * Gets edges adjacent to this vertex
         * @return adjacent edges
         */
        inline const std::vector<Edge<T> *> &getEdges() const
        {
                return edges_;
        }

        /**
         * Gets a vector of the vertices adjacent to this vertex
         * @return adjacent vertices
         */
        std::vector<Vertex<T> *> getAdjacentVertices() const
        {
                std::vector<Vertex<T> *> vertices;

                for (const auto &edge : edges_) {
                        if (edge == nullptr) {
                                continue;
                        }

                        // check both source and target because we have a undirected graph
                        if (edge->getSource() != this) {
                                vertices.push_back(edge->getSource());
                        }

                        if (edge->getTarget() != this) {
                                vertices.push_back(edge->getTarget());
                        }
                }

                return vertices;
        }

private:
        /**
         * Adds the given edge to this vertex's edge list
         * @param pEdge edge to add
         */
        inline void addEdge(Edge<T> *edge)
        {
                edges_.push_back(edge);
        }

}; // Vertex



//////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////

class ScanManager
{
private:
        LocalizedRangeScanMap scans_;
        LocalizedRangeScanVector running_scans_;
        LocalizedRangeScan *last_scan_;
        uint32_t next_scan_id_;

        uint32_t running_buffer_maximum_size_;
        double running_buffer_maximum_distance_;

public:
        ScanManager() 
        : last_scan_(nullptr),
        next_scan_id_(0)
        {
        }

        ScanManager(uint32_t running_buffer_maximum_size, double running_buffer_maximum_distance)
            : last_scan_(nullptr),
              next_scan_id_(0),
              running_buffer_maximum_size_(running_buffer_maximum_size),
              running_buffer_maximum_distance_(running_buffer_maximum_distance)
        {
        }

        /**
         * Gets all scans of all devices
         * @return all scans of all devices
         */
        std::vector<LocalizedRangeScan *> getAllScans()
        {
                std::vector<LocalizedRangeScan *> scans;
                scans.reserve(scans_.size());

                for (const auto &scan : scans_)
                {
                        scans.push_back(scan.second);
                }

                return scans;
        }


        inline void addRunningScan(LocalizedRangeScan *scan)
        {
                running_scans_.push_back(scan);

                // vector has at least one element (first line of this function), so this is valid
                Pose2 front_scan_pose = running_scans_.front()->getSensorPose();
                Pose2 back_scan_pose = running_scans_.back()->getSensorPose();

                // cap vector size and remove all scans from front of vector that are too far from end of vector
                double squared_distance = front_scan_pose.getSquaredDistance(back_scan_pose);
                while (running_scans_.size() > running_buffer_maximum_size_ ||
                       squared_distance > math::Square(running_buffer_maximum_distance_) - KT_TOLERANCE) {
                        // remove front of running scans
                        running_scans_.erase(running_scans_.begin());

                        // recompute stats of running scans
                        front_scan_pose = running_scans_.front()->getSensorPose();
                        back_scan_pose = running_scans_.back()->getSensorPose();
                        squared_distance = front_scan_pose.getSquaredDistance(back_scan_pose);
                }
        }

        inline LocalizedRangeScanVector getRunningScans()
        {
                return running_scans_;
        }

        /**
         * Gets scan from given device with given ID
         * @param rSensorName
         * @param scanNum
         * @return localized range scan
         */
        LocalizedRangeScan *getScan(int32_t scan_index)
        {
                LocalizedRangeScanMap::iterator it = scans_.find(scan_index);
                if (it != scans_.end()) {
                        return it->second;
                } else {
                        return nullptr;
                }
        }

        inline void setLastScan(LocalizedRangeScan *scan)
        {
                last_scan_ = scan;
        }

        
        inline LocalizedRangeScan *getLastScan()
        {
                return last_scan_;
        }


        inline void addScan(LocalizedRangeScan *scan) 
        {
                // assign unique scan id
                scan->setScanId(next_scan_id_);
                // add to scan buffer
                scans_.emplace(next_scan_id_, scan);
                next_scan_id_++;
        }

        void setRunningScanBufferSize(uint32_t scan_buffer_size)
        {
                running_buffer_maximum_size_ = scan_buffer_size;
        }

        void setRunningScanBufferMaximumDistance(double scan_buffer_max_distance)
        {
                running_buffer_maximum_distance_ = scan_buffer_max_distance;
        }



}; // ScanManager

///////////////////////////////////////////////////////////////////////

// A LinkInfo object contains the requisite information for the "spring"
// that links two scans together--the pose difference and the uncertainty
// (represented by a covariance matrix).

class LinkInfo
{
private:
        Pose2 pose1_;
        Pose2 pose2_;
        Pose2 pose_difference_;
        Matrix3d covariance_;

public:
        LinkInfo()
        {
        }

        LinkInfo(const Pose2 &rPose1, const Pose2 &rPose2, const Matrix3d &rCovariance)
        {
                Update(rPose1, rPose2, rCovariance);
        }
public:
        /**
         * Changes the link information to be the given parameters
         * @param rPose1
         * @param rPose2
         * @param rCovariance
         */
        void Update(const Pose2 &pose1, const Pose2 &pose2, const Matrix3d &covariance)
        {
                pose1_ = pose1;
                pose2_ = pose2;

                // transform second pose into the coordinate system of the first pose
                pose_difference_ = Pose2::transformPose(pose1.inverse(), pose2);

                // transform covariance into reference of first pose
                Matrix3d rotation_matrix = Eigen::AngleAxisd(
                        pose1.getHeading(), 
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();
                covariance_ = rotation_matrix * covariance * rotation_matrix.transpose();
        }

        /**
         * Gets the pose difference
         * @return pose difference
         */
        inline const Pose2 &getPoseDifference()
        {
                return pose_difference_;
        }

        /**
         * Gets the link covariance
         * @return link covariance
         */
        inline const Matrix3d &getCovariance()
        {
                return covariance_;
        }
}; // LinkInfo

///////////////////////////////////////////////////////////////////////

/**
 * Represents an object in a graph
 */


///////////////////////////////////////////////////////////////////////

/**
 * Represents an edge in a graph
 */


///////////////////////////////////////////////////////////////////////

template<typename T>
class Graph
{        
public:
        typedef std::map<int, std::unique_ptr<Vertex<T>>> VertexMap;
protected:
        VertexMap vertices_;
        std::vector<std::unique_ptr<Edge<T>>> edges_;
public:
        Graph()
        {
        }
public:
        /**
         * Adds and indexes the given vertex into the map using the given name
         * @param rName
         * @param pVertex
         */
        inline void addVertex(std::unique_ptr<Vertex<T>> vertex)
        {
                int key = vertex->getObject()->getScanId();
                vertices_.emplace(key, std::move(vertex));
        }

        /**
         * Adds an edge to the graph
         * @param pEdge
         */
        inline void addEdge(std::unique_ptr<Edge<T>> edge)
        {
                edges_.push_back(std::move(edge));
        }

        /**
         * Gets the edges of this graph
         * @return graph edges
         */
        inline const std::vector<std::unique_ptr<Edge<T>>> &getEdges() const
        {
                return edges_;
        }

        /**
         * Gets the vertices of this graph
         * @return graph vertices
         */
        inline const VertexMap &getVertices() const
        {
                return vertices_;
        }

}; // Graph

///////////////////////////////////////////////////////////////////////

class CorrelationGrid : public Grid<uint8_t>
{
private:
        /**
         * The point readings are smeared by this value in X and Y to create a smoother response.
         * Default value is 0.03 meters.
         */
        double smear_deviation_;

        // Size of one side of the kernel
        int32_t kernel_size_;

        // Cached kernel for smearing
        std::unique_ptr<uint8_t[]> kernel_;

        // region of interest
        Rectangle2<int32_t> roi_;

public:
        CorrelationGrid()
        {
        }

        /**
         * Constructs a correlation grid of given size and parameters
         * @param width
         * @param height
         * @param borderSize
         * @param resolution
         * @param smearDeviation
         */
        CorrelationGrid(
            uint32_t width, uint32_t height, uint32_t borderSize,
            double resolution, double smearDeviation)
            : Grid<uint8_t>(width + borderSize * 2, height + borderSize * 2),
              smear_deviation_(smearDeviation),
              kernel_(nullptr)
        {
                getCoordinateConverter()->setScale(1.0 / resolution);

                // setup region of interest
                roi_ = Rectangle2<int32_t>(borderSize, borderSize, width, height);

                // calculate kernel
                calculateKernel();
        }

        static std::unique_ptr<CorrelationGrid> createGrid(
                int32_t width,
                int32_t height,
                double resolution,
                double smear_deviation)
        {
                assert(resolution != 0.0);

                // +1 in case of roundoff
                uint32_t border_size = getHalfKernelSize(smear_deviation, resolution) + 1;

                std::unique_ptr<CorrelationGrid> grid = std::make_unique<CorrelationGrid>(width, height, border_size, resolution,
                                                             smear_deviation);

                return grid;
        }

        inline const Rectangle2<int32_t> &getROI() const
        {
                return roi_;
        }

        /**
         * Gets the index into the data pointer of the given grid coordinate
         * @param rGrid
         * @param boundaryCheck
         * @return grid index
         */
        virtual int32_t getGridIndex(const Vector2i &grid, bool boundary_check = true) const
        {
                int32_t x = grid.x() + roi_.getX();
                int32_t y = grid.y() + roi_.getY();

                return Grid<uint8_t>::getGridIndex(Vector2i(x, y), boundary_check);
        }

        /**
         * Smear cell if the cell at the given point is marked as "occupied"
         * @param rGridPoint
         */
        inline void smearPoint(const Vector2i &grid_point)
        {
                assert(kernel_ != nullptr);

                int grid_index = getGridIndex(grid_point);
                if (getDataPointer()[grid_index] != GRIDSTATES_OCCUPIED) {
                        return;
                }

                int32_t half_kernel = kernel_size_ / 2;

                // apply kernel
                for (int32_t j = -half_kernel; j <= half_kernel; j++) {
                        uint8_t *grid_adr = getDataPointer(
                                Vector2i(grid_point.x(), grid_point.y() + j));

                        int32_t kernel_constant = (half_kernel) + kernel_size_ * (j + half_kernel);

                        // if a point is on the edge of the grid, there is no problem
                        // with running over the edge of allowable memory, because
                        // the grid has margins to compensate for the kernel size
                        for (int32_t i = -half_kernel; i <= half_kernel; i++) {
                                int32_t kernel_array_index = i + kernel_constant;

                                uint8_t kernel_value = kernel_[kernel_array_index];
                                if (kernel_value > grid_adr[i]) {
                                        // kernel value is greater, so set it to kernel value
                                        grid_adr[i] = kernel_value;
                                }
                        }
                }
        }

protected:

        /**
         * Computes the kernel half-size based on the smear distance and the grid resolution.
         * Computes to two standard deviations to get 95% region and to reduce aliasing.
         * @param smearDeviation
         * @param resolution
         * @return kernel half-size based on the parameters
         */
        static int32_t getHalfKernelSize(double smear_deviation, double resolution)
        {
                assert(resolution != 0.0);

                return static_cast<int32_t>(std::round(2.0 * smear_deviation / resolution));
        }

        /**
         * Sets up the kernel for grid smearing.
         */
        virtual void calculateKernel()
        {
                double resolution = getResolution();

                assert(resolution != 0.0);
                assert(smear_deviation_ != 0.0);

                // min and max distance deviation for smearing;
                // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
                const double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
                const double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

                // check if given value too small or too big
                if (!math::InRange(smear_deviation_, MIN_SMEAR_DISTANCE_DEVIATION,
                                   MAX_SMEAR_DISTANCE_DEVIATION))
                {
                        std::stringstream error;
                        error << "Mapper Error:  Smear deviation too small:  Must be between " << MIN_SMEAR_DISTANCE_DEVIATION << " and " << MAX_SMEAR_DISTANCE_DEVIATION;
                        throw std::runtime_error(error.str());
                }

                // NOTE:  Currently assumes a two-dimensional kernel

                // +1 for center
                kernel_size_ = 2 * getHalfKernelSize(smear_deviation_, resolution) + 1;

                // allocate kernel
                kernel_ = std::make_unique<uint8_t[]>(kernel_size_ * kernel_size_);
                if (kernel_ == nullptr) {
                        throw std::runtime_error("Unable to allocate memory for kernel!");
                }

                // calculate kernel
                int32_t half_kernel = kernel_size_ / 2;
                for (int32_t i = -half_kernel; i <= half_kernel; i++)
                {
                        for (int32_t j = -half_kernel; j <= half_kernel; j++) {
                                #ifdef WIN32
                                double distance_from_mean = _hypot(i * resolution, j * resolution);
                                #else
                                double distance_from_mean = hypot(i * resolution, j * resolution);
                                #endif
                                double z = exp(-0.5 * pow(distance_from_mean / smear_deviation_, 2));

                                uint32_t kernel_value = static_cast<uint32_t>(math::Round(z * GRIDSTATES_OCCUPIED));
                                assert(math::IsUpTo(kernel_value, static_cast<uint32_t>(255)));

                                int kernelArrayIndex = (i + half_kernel) + kernel_size_ * (j + half_kernel);
                                kernel_[kernelArrayIndex] = static_cast<uint8_t>(kernel_value);
                        }
                }
        }

}; // CorrelationGrid


///////////////////////////////////////////////////////////////////////
class Mapper;

class ScanMatcher
{
private:
        Mapper *mapper_;
        std::unique_ptr<CorrelationGrid> correlation_grid_;
        std::unique_ptr<Grid<double>> search_space_probs_;
        std::unique_ptr<GridIndexLookup<uint8_t>> grid_lookup_;
        std::unique_ptr<std::pair<double, Pose2>[]> pose_response_;
        std::vector<double> x_poses_;
        std::vector<double> y_poses_;
        Pose2 search_center_;
        double search_angle_offset_;
        uint32_t n_angles_;
        double search_angle_resolution_;
        bool do_penalize_;

        /**
         * Marks cells where scans' points hit as being occupied
         * @param scan scans whose points will mark cells in grid as being occupied
         * @param view_point do not add points that belong to scans "opposite" the view point
         */
        void addScans(const LocalizedRangeScanVector &scan, Eigen::Vector2d view_point);
        void addScans(const LocalizedRangeScanMap &scans, Eigen::Vector2d view_point);

        /**
         * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
         * @param pScan scan whose points will mark cells in grid as being occupied
         * @param viewPoint do not add points that belong to scans "opposite" the view point
         * @param doSmear whether the points will be smeared
         */
        void addScan(
            LocalizedRangeScan *scan, const Eigen::Vector2d &viewpoint,
            bool do_smear = true);

        /**
         * Compute which points in a scan are on the same side as the given viewpoint
         * @param pScan
         * @param rViewPoint
         * @return points on the same side
         */
        PointVectorDouble findValidPoints(
                LocalizedRangeScan *scan,
                const Eigen::Vector2d &viewpoint) const;

        /**
         * Get response at given position for given rotation (only look up valid points)
         * @param angleIndex
         * @param gridPositionIndex
         * @return response
         */
        double getResponse(uint32_t angle_index, int32_t grid_position_index) const;

public:
        ScanMatcher()
        {
        }

        /**
         * Default constructor
         */
        explicit ScanMatcher(Mapper *mapper)
            : mapper_(mapper),
              correlation_grid_(nullptr),
              search_space_probs_(nullptr),
              grid_lookup_(nullptr),
              pose_response_(nullptr),
              do_penalize_(false)
        {
        }

        /**
         * Parallelize scan matching
         */
        void operator()(const double &y) const;

        /**
         * Create a scan matcher with the given parameters
         */
        static std::unique_ptr<ScanMatcher> create(
                Mapper *mapper,
                double search_size,
                double resolution,
                double smear_deviation,
                double range_threshold);

        /**
         * Match given scan against set of scans
         * @param pScan scan being scan-matched
         * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
         * @param rMean output parameter of mean (best pose) of match
         * @param rCovariance output parameter of covariance of match
         * @param doPenalize whether to penalize matches further from the search center
         * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
         * @return strength of response
         */
        template <class T = std::vector<LocalizedRangeScan *>>
        double matchScan(
                LocalizedRangeScan *scan,
                const T &base_scans,
                Pose2 &mean, Eigen::Matrix3d &covariance,
                bool do_penalize = true,
                bool do_refine_match = true);

        /**
         * Finds the best pose for the scan centering the search in the correlation grid
         * at the given pose and search in the space by the vector and angular offsets
         * in increments of the given resolutions
         * @param pScan scan to match against correlation grid
         * @param rSearchCenter the center of the search space
         * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
         * @param rSearchSpaceResolution how fine a granularity to search in the search space
         * @param searchAngleOffset searches poses in the angles offset by this angle around search center
         * @param searchAngleResolution how fine a granularity to search in the angular search space
         * @param doPenalize whether to penalize matches further from the search center
         * @param rMean output parameter of mean (best pose) of match
         * @param rCovariance output parameter of covariance of match
         * @param doingFineMatch whether to do a finer search after coarse search
         * @return strength of response
         */
        double correlateScan(
                LocalizedRangeScan *scan,
                const Pose2 &search_center,
                const Vector2d &search_space_offset,
                const Vector2d &search_space_resolution,
                double search_angle_offset,
                double search_angle_resolution,
                bool do_penalize,
                Pose2 &mean,
                Matrix3d &covariance,
                bool doing_fine_match);

        /**
         * Computes the positional covariance of the best pose
         * @param rBestPose
         * @param bestResponse
         * @param rSearchCenter
         * @param rSearchSpaceOffset
         * @param rSearchSpaceResolution
         * @param searchAngleResolution
         * @param rCovariance
         */
        void computePositionalCovariance(
            const Pose2 &best_pose,
            double best_response,
            const Pose2 &search_center,
            const Vector2d &search_space_offset,
            const Vector2d &search_space_resolution,
            double search_angle_resolution,
            Matrix3d &covariance);

        /**
         * Computes the angular covariance of the best pose
         * @param rBestPose
         * @param bestResponse
         * @param rSearchCenter
         * @param searchAngleOffset
         * @param searchAngleResolution
         * @param rCovariance
         */
        void computeAngularCovariance(
            const Pose2 &best_pose,
            double best_response,
            const Pose2 &search_center,
            double search_angle_offset,
            double search_angle_resolution,
            Matrix3d &covariance);
}; // ScanMatcher

///////////////////////////////////////////////////////////////////////
template <typename T>
class Visitor;

/**
* Graph traversal algorithm
*/
template<typename T>
class GraphTraversal
{
protected:
        Graph<T> *graph_;

public:
        GraphTraversal()
        {
        }

        explicit GraphTraversal(Graph<T> *pGraph)
            : graph_(pGraph)
        {
        }

public:
        virtual std::vector<T *> traverseForScans(Vertex<T> *start_vertex, Visitor<T> *visitor) = 0;
        virtual std::vector<Vertex<T> *> traverseForVertices(
                Vertex<T> *start_vertex,
                Visitor<T> *visitor) = 0;

}; // GraphTraversal<T>

///////////////////////////////////////////////////////////////////////

template<typename T>
class BreadthFirstTraversal : public GraphTraversal<T>
{
public:
        /**
         * Constructs a breadth-first traverser for the given graph
         */
        BreadthFirstTraversal()
        {
        }
        explicit BreadthFirstTraversal(Graph<T> *graph)
                : GraphTraversal<T>(graph)
        {
        }

public:
        /**
         * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
         * @param pStartVertex
         * @param pVisitor
         * @return visited vertice scans
         */
        virtual std::vector<T *> traverseForScans(Vertex<T> *start_vertex, Visitor<T> *visitor)
        {
                std::vector<Vertex<T> *> valid_vertices = traverseForVertices(start_vertex, visitor);

                std::vector<T *> objects;
                for (auto& vertex : valid_vertices) {
                        objects.push_back(vertex->getObject());
                }

                return objects;
        }

        /**
         * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
         * @param pStartVertex
         * @param pVisitor
         * @return visited vertices
         */
        virtual std::vector<Vertex<T> *> traverseForVertices(
                Vertex<T> *start_vertex,
                Visitor<T> *visitor)
        {
                std::queue<Vertex<T> *> to_visit;
                std::set<Vertex<T> *> seen_vertices;
                std::vector<Vertex<T> *> valid_vertices;

                to_visit.push(start_vertex);
                seen_vertices.insert(start_vertex);

                do {
                        Vertex<T> *next = to_visit.front();
                        to_visit.pop();

                        if (next != nullptr && visitor->visit(next)) {
                                // vertex is valid, explore neighbors
                                valid_vertices.push_back(next);

                                std::vector<Vertex<T> *> adjacent_vertices = next->getAdjacentVertices();
                                for (auto &vertex : adjacent_vertices) {
                                        if (seen_vertices.find(vertex) == seen_vertices.end()) {
                                                to_visit.push(vertex);
                                                seen_vertices.insert(vertex);
                                        }
                                }
                        }
                } while (to_visit.empty() == false);

                return valid_vertices;
        }


}; // BreadthFirstTraversal<T>

///////////////////////////////////////////////////////////////////////

/**
 * Visitor class
 */
template <typename T>
class Visitor
{
public:
        /**
         * Applies the visitor to the vertex
         * @param pVertex
         * @return true if the visitor accepted the vertex, false otherwise
         */
        virtual bool visit(Vertex<T> *vertex) = 0;
}; // Visitor<T>

///////////////////////////////////////////////////////////////////////

class NearScanVisitor : public Visitor<LocalizedRangeScan>
{
protected:
        Pose2 center_pose_;
        double max_distance_squared_;
        bool use_scan_barycenter_;
public:
        NearScanVisitor(LocalizedRangeScan *scan, double max_distance, bool use_scan_barycenter)
            : max_distance_squared_(math::Square(max_distance)),
              use_scan_barycenter_(use_scan_barycenter)
        {
                center_pose_ = scan->getReferencePose(use_scan_barycenter);
        }

        virtual bool visit(Vertex<LocalizedRangeScan> *vertex)
        {
                try {
                        LocalizedRangeScan *scan = vertex->getObject();
                        Pose2 pose = scan->getReferencePose(use_scan_barycenter_);
                        double squared_distance = pose.getSquaredDistance(center_pose_);
                        return squared_distance <= max_distance_squared_ - KT_TOLERANCE;
                } catch (...) {
                        // relocalization vertex elements missing
                        std::cout << "Unable to visit valid vertex elements!" << std::endl;
                        return false;
                }
        }
}; // NearScanVisitor

///////////////////////////////////////////////////////////////////////

class MapperGraph : public Graph<LocalizedRangeScan>
{
private:
        /**
         * Mapper of this graph
         */
        Mapper *mapper_;

        /**
         * Scan matcher for loop closures
         */
        std::unique_ptr<ScanMatcher> loop_scan_matcher_;

        /**
         * Traversal algorithm to find near linked scans
         */
        std::unique_ptr<GraphTraversal<LocalizedRangeScan>> traversal_;

public:
        MapperGraph()
        {
        }

        /**
         * Graph for graph SLAM
         * @param pMapper
         * @param rangeThreshold
         */
        MapperGraph(Mapper *mapper, double range_threshold);

        /**
         * Adds a vertex representing the given scan to the graph
         * @param pScan
         */
        Vertex<LocalizedRangeScan> *addVertex(LocalizedRangeScan *scan);

        /**
         * Creates an edge between the source scan vertex and the target scan vertex if it
         * does not already exist; otherwise return the existing edge
         * @param pSourceScan
         * @param pTargetScan
         * @param rIsNewEdge set to true if the edge is new
         * @return edge between source and target scan vertices
         */
        Edge<LocalizedRangeScan> *addEdge(
                LocalizedRangeScan *source_scan,
                LocalizedRangeScan *target_scan,
                bool &is_new_edge);

        /**
         * Link scan to last scan and nearby chains of scans
         * @param pScan
         * @param rCovariance uncertainty of match
         */
        void addEdges(LocalizedRangeScan *scan, const Matrix3d &covariance);


        bool tryCloseLoop(LocalizedRangeScan *scan);

        /**
         * Optimizes scan poses
         */
        void correctPoses();

        /**
         * Find "nearby" (no further than given distance away) scans through graph links
         * @param pScan
         * @param maxDistance
         */
        LocalizedRangeScanVector findNearLinkedScans(LocalizedRangeScan *scan, double max_distance);

private:
        /**
         * Adds an edge between the two scans and labels the edge with the given mean and covariance
         * @param pFromScan
         * @param pToScan
         * @param rMean
         * @param rCovariance
         */
        void linkScans(
            LocalizedRangeScan *from_scan,
            LocalizedRangeScan *to_scan,
            const Pose2 &mean,
            const Matrix3d &covariance);

        /**
         * Finds the closest scan in the vector to the given pose
         * @param rScans
         * @param rPose
         */
        LocalizedRangeScan *getClosestScanToPose(
            const LocalizedRangeScanVector &scans,
            const Pose2 &pose) const;

        /**
         * Tries to find a chain of scan from the given device starting at the
         * given scan index that could possibly close a loop with the given scan
         * @param pScan
         * @param rSensorName
         * @param rStartNum
         * @return chain that can possibly close a loop with given scan
         */
        LocalizedRangeScanVector FindPossibleLoopClosure(
            LocalizedRangeScan *pScan,
            uint32_t &rStartNum);

        /**
         * Link the chain of scans to the given scan by finding the closest scan in the chain to the given scan
         * @param rChain
         * @param pScan
         * @param rMean
         * @param rCovariance
         */
        void linkChainToScan(
                const LocalizedRangeScanVector &chain,
                LocalizedRangeScan *scan,
                const Pose2 &mean,
                const Matrix3d &covariance);

        /**
         * Find nearby chains of scans and link them to scan if response is high enough
         * @param pScan
         * @param rMeans
         * @param rCovariances
         */
        void linkNearChains(
            LocalizedRangeScan *scan, Pose2Vector &means,
            std::vector<Matrix3d> &covariances);

        /**
         * Find chains of scans that are close to given scan
         * @param pScan
         * @return chains of scans
         */
        std::vector<LocalizedRangeScanVector> findNearChains(LocalizedRangeScan *scan);

        /**
         * Compute mean of poses weighted by covariances
         * @param rMeans
         * @param rCovariances
         * @return weighted mean
         */
        Pose2 computeWeightedMean(
            const Pose2Vector &means,
            const std::vector<Matrix3d> &covariances) const;

        /**
         * Gets the vertex associated with the given scan
         * @param pScan
         * @return vertex of scan
         */
        inline Vertex<LocalizedRangeScan> *getVertex(LocalizedRangeScan *scan)
        {
                std::map<int, std::unique_ptr<Vertex<LocalizedRangeScan>>>::iterator it = vertices_.find(
                    scan->getScanId());
                if (it != vertices_.end())
                {
                        return it->second.get();
                }
                else
                {
                        std::cout << "GetVertex: Failed to get vertex, idx " << scan->getScanId() << " is not in m_Vertices." << std::endl;
                        return nullptr;
                }
        }

}; // MapperGraph

///////////////////////////////////////////////////////////////////////

class ScanSolver
{
public:
        /**
         * Vector of id-pose pairs
         */
        typedef std::vector<std::pair<int32_t, Pose2>> IdPoseVector;

        ScanSolver()
        {
        }

        /**
         * Solve!
         */
        virtual void compute() = 0;

        /**
         * Adds a node to the solver
         */
        virtual void addNode(Vertex<LocalizedRangeScan> * /*vertex*/)
        {
        }

        /**
         * Adds a constraint to the solver
         */
        virtual void addConstraint(Edge<LocalizedRangeScan> * /*pEdge*/)
        {
        }

        /**
         * Resets the solver
         */
        virtual void clear()
        {
        }

        /**
         * Get graph stored
         */
        virtual std::unordered_map<int, Eigen::Vector3d> *getGraph()
        {
                std::cout << "getGraph method not implemented for this solver type. Graph visualization unavailable." << std::endl;
                return nullptr;
        }

        /**
         * Get corrected poses after optimization
         * @return optimized poses
         */
        virtual const IdPoseVector &getCorrections() const = 0;

}; // ScanSolver

///////////////////////////////////////////////////////////////////////


class Mapper 
{
        friend class ScanMatcher;
        friend class MapperGraph;
protected:
        // state
        bool initialized_;

        // parameters
        double minimum_travel_distance_;
        double minimum_travel_heading_;
        /**
         * Scan buffer size is the length of the scan chain stored for scan matching.
         * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
         * The idea is to get an area approximately 20 meters long for scan matching.
         * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
         * should be 20 / 0.3 = 67.)
         * Default value is 67.
         */
        uint32_t scan_buffer_size_;
        /**
         * Scan buffer maximum scan distance is the maximum distance between the first and last scans
         * in the scan chain stored for matching.
         * Default value is 20.0.
         */
        double scan_buffer_maximum_scan_distance_;

        ////////////////////////////////////////////////////////////
        // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
        // then not many beams will hit the cell!

        // Number of beams that must pass through a cell before it will be considered to be occupied
        // or unoccupied.  This prevents stray beams from messing up the map.
        uint32_t min_pass_through_;

        // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
        double occupancy_threshold_;

        // The range of angles to search during a coarse search and a finer search
        double fine_search_angle_offset_;
        double coarse_search_angle_offset_;

        // whether to increase the search space if no good matches are initially found
        bool use_response_expansion;

        // Resolution of angles to search during a coarse search
        double coarse_angle_resolution_;

        // Variance of penalty for deviating from odometry when scan-matching.
        // The penalty is a multiplier (less than 1.0) is a function of the
        // delta of the scan position being tested and the odometric pose
        double distance_variance_penalty_;
        double angle_variance_penalty_;

        // Minimum value of the penalty multiplier so scores do not
        // become too small
        double minimum_angle_penalty_;
        double minimum_distance_penalty_;

        /**
         * The size of the search grid used by the matcher.
         * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
         */
        double correlation_search_space_dimension_;

        /**
         * The resolution (size of a grid cell) of the correlation grid.
         * Default value is 0.01 meters.
         */
        double correlation_search_space_resolution_;

        /**
         * The point readings are smeared by this value in X and Y to create a smoother response.
         * Default value is 0.03 meters.
         */
        double correlation_search_space_smear_deviation_;

        /**
         * Default value is true.
         */
        bool use_scan_barycenter_;

        /**
         * Scans are linked only if the correlation response value is greater than this value.
         * Default value is 0.4
         */
        double link_match_minimum_response_fine_;

        /**
         * Maximum distance between linked scans.  Scans that are farther apart will not be linked
         * regardless of the correlation response value.
         * Default value is 6.0 meters.
         */
        double link_scan_maximum_distance_;

        /**
         * Scans less than this distance from the current position will be considered for a match
         * in loop closure.
         * Default value is 4.0 meters.
         */
        double loop_search_maximum_distance_;

        /**
         * When the loop closure detection finds a candidate it must be part of a large
         * set of linked scans. If the chain of scans is less than this value we do not attempt
         * to close the loop.
         * Default value is 10.
         */
        uint32_t loop_match_minimum_chain_size_;

        /**
         * The co-variance values for a possible loop closure have to be less than this value
         * to consider a viable solution. This applies to the coarse search.
         * Default value is 0.16.
         */
        double loop_match_maximum_variance_coarse_;

        /**
         * If response is larger then this, then initiate loop closure search at the coarse resolution.
         * Default value is 0.7.
         */
        double loop_match_minimum_response_coarse_;

        /**
         * If response is larger then this, then initiate loop closure search at the fine resolution.
         * Default value is 0.7.
         */
        double loop_match_minimum_response_fine_;

        /**
         * The size of the search grid used by the matcher.
         * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
         */
        double loop_search_space_dimension_;

        /**
         * The resolution (size of a grid cell) of the correlation grid.
         * Default value is 0.01 meters.
         */
        double loop_search_space_resolution_;

        /**
         * The point readings are smeared by this value in X and Y to create a smoother response.
         * Default value is 0.03 meters.
         */
        double loop_search_space_smear_deviation_;

        std::unique_ptr<ScanManager> scan_manager_;
        std::unique_ptr<ScanMatcher> scan_matcher_;
        std::unique_ptr<MapperGraph> graph_;
        std::unique_ptr<ScanSolver> scan_optimizer_;

protected:
        bool hasMovedEnough(LocalizedRangeScan *scan, LocalizedRangeScan *last_scan) const;

public:
        Mapper()
        : scan_manager_(nullptr),
        initialized_(false)
        {
        }

        void initialize(double range_threshold)
        {
                if (initialized_ == true) {
                        return;
                }

                // create sequential scan and loop matcher

                scan_matcher_ = ScanMatcher::create(
                        this,
                        correlation_search_space_dimension_,
                        correlation_search_space_resolution_,
                        correlation_search_space_smear_deviation_,
                        range_threshold);
                assert(scan_matcher_);

                scan_manager_ = std::make_unique<ScanManager>(scan_buffer_size_, scan_buffer_maximum_scan_distance_);

                graph_ = std::make_unique<MapperGraph>(this, range_threshold);

                initialized_ = true;
        }

        template <class NodeT>
        void configure(const NodeT &node);

        inline double getMinimumTravelDistance() const {
                return minimum_travel_distance_;
        }

        /**
         * @return Minimum travel heading in radians 
         */
        inline double getMinimumTravelHeading() const {
                return minimum_travel_heading_;
        }

        void setScanSolver(std::unique_ptr<ScanSolver> scan_optimizer)
        {
                scan_optimizer_ = std::move(scan_optimizer);
        }

        MapperGraph *getGraph() const
        {
                return graph_.get();
        }

        ScanSolver *getScanSolver()
        {
                return scan_optimizer_.get();
        }

        /**
         * Gets the device manager
         * @return device manager
         */
        inline ScanManager *getScanManager() const
        {
                return scan_manager_.get();
        }

        /**
         * Process a localized range scan for incorporation into the map.  The scan must
         * be identified with a range finder device.  Once added to a map, the corrected pose information in the
         * localized scan will be updated to the correct pose as determined by the mapper.
         *
         * @param scan A localized range scan that has pose information associated directly with the scan data.  The pose
         * is that of the range device originating the scan.  Note that the mapper will set corrected pose
         * information in the scan object.
         *
         * @return true if the scan was added successfully, false otherwise
         */
        bool process(LocalizedRangeScan *scan, Eigen::Matrix3d *covariance = nullptr);

        /**
         * Returns all processed scans added to the mapper.
         * NOTE: The returned scans have their corrected pose updated.
         * @return list of scans received and processed by the mapper. If no scans have been processed,
         * return an empty list.
         */
        const std::vector<LocalizedRangeScan *> getAllProcessedScans() const
        {
                std::vector<LocalizedRangeScan *> all_scans;

                if (scan_manager_ != nullptr)
                {
                        all_scans = scan_manager_->getAllScans();
                }

                return all_scans;
        }

        // get occupancy grid from scans
        OccupancyGrid *getOccupancyGrid(const double &resolution)
        {
                std::cout << "resolution1 is " << resolution << std::endl;
                return OccupancyGrid::createFromScans(
                        getAllProcessedScans(),
                        resolution,
                        (uint32_t)getParamMinPassThrough(),
                        (double)getParamOccupancyThreshold());
        }

        uint32_t getParamMinPassThrough()
        {
                return min_pass_through_;
        }

        double getParamOccupancyThreshold()
        {
                return occupancy_threshold_;
        }

public:
        void setUseScanBarycenter(bool b) 
        {
                use_scan_barycenter_ = b;
        }

        void setScanBufferSize(int i)
        {
                scan_buffer_size_ = (uint32_t)i;
        }

        void setScanBufferMaximumScanDistance(double d)
        {
                scan_buffer_maximum_scan_distance_ = d;
        }

        void setLinkMatchMinimumResponseFine(double d)
        {
                link_match_minimum_response_fine_ = d;
        }

        void setLinkScanMaximumDistance(double d)
        {
                link_scan_maximum_distance_ = d;
        }

        void setLoopSearchMaximumDistance(double d)
        {
                loop_search_maximum_distance_ = d;
                
        }

        void setLoopMatchMinimumChainSize(int i)
        {
                loop_match_minimum_chain_size_ = (uint32_t)i;
        }

        void setLoopMatchMaximumVarianceCoarse(double d)
        {
               loop_match_maximum_variance_coarse_ = d;
        }

        void setLoopMatchMinimumResponseCoarse(double d)
        {
               loop_match_minimum_response_coarse_ = d; 
        }

        void setLoopMatchMinimumResponseFine(double d)
        {
                loop_match_minimum_response_fine_ = d;
        }

        // Correlation Parameters - Correlation Parameters
        void setCorrelationSearchSpaceDimension(double d)
        {
               correlation_search_space_dimension_ = d;
        }

        void setCorrelationSearchSpaceResolution(double d)
        {
                correlation_search_space_resolution_ = d;
        }

        void setCorrelationSearchSpaceSmearDeviation(double d)
        {
                correlation_search_space_smear_deviation_ = d;
        }

        // Correlation Parameters - Loop Closure Parameters
        void setLoopSearchSpaceDimension(double d)
        {
                loop_search_space_dimension_ = d;
        }

        void setLoopSearchSpaceResolution(double d)
        {
               loop_search_space_resolution_ = d;
        }

        void setLoopSearchSpaceSmearDeviation(double d)
        {
                loop_search_space_smear_deviation_ = d;
        }

        // Scan Matcher Parameters
        void setDistanceVariancePenalty(double d)
        {
                distance_variance_penalty_ = math::Square(d);
        }

        void setAngleVariancePenalty(double d)
        {
                angle_variance_penalty_ = math::Square(d);
        }

        void setFineSearchAngleOffset(double d)
        {
                fine_search_angle_offset_ = d;
        }

        void setCoarseSearchAngleOffset(double d)
        {
                coarse_search_angle_offset_ = d;
        }

        void setCoarseAngleResolution(double d)
        {
                coarse_angle_resolution_ = d;
        }

        void setMinimumAnglePenalty(double d)
        {
                minimum_angle_penalty_ = d;
        }

        void setMinimumDistancePenalty(double d)
        {
                minimum_distance_penalty_ = d;
        }

        void setUseResponseExpansion(bool b)
        {
                use_response_expansion = b;
        }

        void setMinPassThrough(int i)
        {
                min_pass_through_ = (uint32_t)i;
        }

        void setOccupancyThreshold(double d)
        {
                occupancy_threshold_ = d;
        }

}; // Mapper

//////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////



///////////////////////////////////////////////////////


/////////////////////////////////////////////////



} // namespace mapper_utils

#endif // KARTO_SDK_MAPPER_HPP