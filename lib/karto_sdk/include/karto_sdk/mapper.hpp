#ifndef KARTO_SDK_MAPPER_HPP
#define KARTO_SDK_MAPPER_HPP

#include <iostream>
#include <unordered_map>
#include <queue>
#include <set>

#include "Eigen/Core"
#include "tbb/parallel_for_each.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "math.hpp"
#include "karto.hpp"


namespace karto
{

typedef std::vector<LocalizedRangeScan *> LocalizedRangeScanVector;
typedef std::map<int, LocalizedRangeScan *> LocalizedRangeScanMap;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

template <typename T>
class Vertex;
class LinkInfo;

template <typename T>
class Edge
{
private:
        Vertex<T> *source_;
        Vertex<T> *target_;
        std::unique_ptr<LinkInfo> label_;

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
         * @param label
         */
        inline void setLabel(std::unique_ptr<LinkInfo> label)
        {
                label_ = std::move(label);
        }

        /**
         * Gets the link info
         * @return link info
         */
        inline LinkInfo *getLabel()
        {
                return label_.get();
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
         * @param edge edge to add
         */
        inline void addEdge(Edge<T> *edge)
        {
                edges_.push_back(edge);
        }

}; // Vertex

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
        ScanManager(uint32_t running_buffer_maximum_size, double running_buffer_maximum_distance)
        : last_scan_(nullptr),
        next_scan_id_(0),
        running_buffer_maximum_size_(running_buffer_maximum_size),
        running_buffer_maximum_distance_(running_buffer_maximum_distance)
        {
        }

        /**
         * Destructor
         */
        virtual ~ScanManager()
        {
                clear();
        }

public:
        /**
         * Deletes data of this buffered device
         */
        void clear()
        {
                scans_.clear();
                running_scans_.clear();
        }

        /**
         * Gets all scans
         * @return all scans as a std::vector
         */
        LocalizedRangeScanVector getAllScans()
        {
                std::vector<LocalizedRangeScan *> scans;
                scans.reserve(scans_.size());

                for (const auto &scan : scans_) {
                        scans.push_back(scan.second);
                }

                return scans;
        }

        /**
         * Gets all scans
         * @return all scans as a std::map<id,scan>
         */
        LocalizedRangeScanMap getAllScansWithId() 
        {
                return scans_;
        }


        inline void addRunningScan(LocalizedRangeScan *scan)
        {
                running_scans_.push_back(scan);

                // vector has at least one element (first line of this function), so this is valid
                Pose2 front_scan_pose = running_scans_.front()->getSensorPose();
                Pose2 back_scan_pose = running_scans_.back()->getSensorPose();

                // cap vector size and remove all scans from front of vector that are too far from end of vector
                double squared_distance = front_scan_pose.computeSquaredDistance(back_scan_pose);
                while (running_scans_.size() > running_buffer_maximum_size_ ||
                       squared_distance > math::Square(running_buffer_maximum_distance_) - KT_TOLERANCE) {
                        // remove front of running scans
                        running_scans_.erase(running_scans_.begin());

                        // recompute stats of running scans
                        front_scan_pose = running_scans_.front()->getSensorPose();
                        back_scan_pose = running_scans_.back()->getSensorPose();
                        squared_distance = front_scan_pose.computeSquaredDistance(back_scan_pose);
                }
        }

        inline LocalizedRangeScanVector getRunningScans()
        {
                return running_scans_;
        }

        /**
         * Gets scan with given ID
         * @param scan_index
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// A LinkInfo object contains the requisite information for the "spring"
// that links two scans together--the pose difference and the uncertainty
// (represented by a covariance matrix).

class LinkInfo
{
private:
        Pose2 pose1_;
        Pose2 pose2_;
        Pose2 pose_difference_;
        Matrix3 covariance_;

public:
        LinkInfo()
        {
        }

        LinkInfo(const Pose2 &pose1, const Pose2 &pose2, const Matrix3 &rCovariance)
        {
                update(pose1, pose2, rCovariance);
        }
public:
        /**
         * Changes the link information to be the given parameters
         * @param pose1
         * @param pose2
         * @param covariance
         */
        void update(const Pose2 &pose1, const Pose2 &pose2, const Matrix3 &covariance)
        {
                pose1_ = pose1;
                pose2_ = pose2;

                // transform second pose into the coordinate system of the first pose
                Transform transform(pose1, Pose2());
                pose_difference_ = transform.transformPose(pose2);

                // transform covariance into reference of first pose
                Matrix3 rotationMatrix;
                rotationMatrix.fromAxisAngle(0, 0, 1, -pose1.getHeading());

                covariance_ = rotationMatrix * covariance * rotationMatrix.transpose();
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
        inline const Matrix3 &getCovariance()
        {
                return covariance_;
        }
}; // LinkInfo

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
         * @param vertex
         */
        inline void addVertex(std::unique_ptr<Vertex<T>> vertex)
        {
                int key = vertex->getObject()->getScanId();
                vertices_.emplace(key, std::move(vertex));
        }

        /**
         * Adds an edge to the graph
         * @param edge
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
            uint32_t width, uint32_t height, uint32_t border_size,
            double resolution, double smear_deviation)
            : Grid<uint8_t>(width + border_size * 2, height + border_size * 2),
              smear_deviation_(smear_deviation),
              kernel_(nullptr)
        {
                getCoordinateConverter()->setScale(1.0 / resolution);

                // setup region of interest
                roi_ = Rectangle2<int32_t>(border_size, border_size, width, height);

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
         * @param grid
         * @param boundary_check
         * @return grid index
         */
        virtual int32_t getGridIndex(const Vector2<int32_t> &grid, bool boundary_check = true) const
        {
                int32_t x = grid.getX() + roi_.getX();
                int32_t y = grid.getY() + roi_.getY();

                return Grid<uint8_t>::getGridIndex(Vector2<int32_t>(x, y), boundary_check);
        }

        /**
         * Smear cell if the cell at the given point is marked as "occupied"
         * @param grid_point
         */
        inline void smearPoint(const Vector2<int32_t> &grid_point)
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
                                Vector2<int32_t>(grid_point.getX(), grid_point.getY() + j));

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
         * @param smear_deviation
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
        void addScans(const LocalizedRangeScanVector &scan, Vector2<double> view_point);
        void addScans(const LocalizedRangeScanMap &scans, Vector2<double> view_point);

        /**
         * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
         * @param scan scan whose points will mark cells in grid as being occupied
         * @param viewpoint do not add points that belong to scans "opposite" the view point
         * @param do_smear whether the points will be smeared
         */
        void addScan(
            LocalizedRangeScan *scan, const Vector2<double> &viewpoint,
            bool do_smear = true);

        /**
         * Compute which points in a scan are on the same side as the given viewpoint
         * @param scan
         * @param viewpoint
         * @return points on the same side
         */
        PointVectorDouble findValidPoints(
                LocalizedRangeScan *scan,
                const Vector2<double> &viewpoint) const;

        /**
         * Get response at given position for given rotation (only look up valid points)
         * @param angle_index
         * @param grid_position_index
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
         * @param scan scan being scan-matched
         * @param base_scans set of scans whose points will mark cells in grid as being occupied
         * @param mean output parameter of mean (best pose) of match
         * @param covariance output parameter of covariance of match
         * @param do_penalize whether to penalize matches further from the search center
         * @param do_refine_match whether to do finer-grained matching if coarse match is good (default is true)
         * @return strength of response
         */
        template <class T = std::vector<LocalizedRangeScan *>>
        double matchScan(
                LocalizedRangeScan *scan,
                const T &base_scans,
                Pose2 &mean, Matrix3 &covariance,
                bool do_penalize = true,
                bool do_refine_match = true);

        /**
         * Finds the best pose for the scan centering the search in the correlation grid
         * at the given pose and search in the space by the vector and angular offsets
         * in increments of the given resolutions
         * @param scan scan to match against correlation grid
         * @param search_center the center of the search space
         * @param serch_space_offset searches poses in the area offset by this vector around search center
         * @param search_space_resolution how fine a granularity to search in the search space
         * @param search_angle_offset searches poses in the angles offset by this angle around search center
         * @param search_angle_resolution how fine a granularity to search in the angular search space
         * @param do_penalize whether to penalize matches further from the search center
         * @param mean output parameter of mean (best pose) of match
         * @param covariance output parameter of covariance of match
         * @param doing_fine_match whether to do a finer search after coarse search
         * @return strength of response
         */
        double correlateScan(
                LocalizedRangeScan *scan,
                const Pose2 &search_center,
                const Vector2<double> &search_space_offset,
                const Vector2<double> &search_space_resolution,
                double search_angle_offset,
                double search_angle_resolution,
                bool do_penalize,
                Pose2 &mean,
                Matrix3 &covariance,
                bool doing_fine_match);

        /**
         * Computes the positional covariance of the best pose
         * @param bset_pose
         * @param bestResponse
         * @param search_center
         * @param serch_space_offset
         * @param search_space_resolution
         * @param searchAngleResolution
         * @param rCovariance
         */
        void computePositionalCovariance(
            const Pose2 &best_pose,
            double best_response,
            const Pose2 &search_center,
            const Vector2<double> &search_space_offset,
            const Vector2<double> &search_space_resolution,
            double search_angle_resolution,
            Matrix3 &covariance);

        /**
         * Computes the angular covariance of the best pose
         * @param bset_pose
         * @param best_response
         * @param search_center
         * @param search_angle_offset
         * @param search_angle_resolution
         * @param covariance
         */
        void computeAngularCovariance(
            const Pose2 &best_pose,
            double best_response,
            const Pose2 &search_center,
            double search_angle_offset,
            double search_angle_resolution,
            Matrix3 &covariance);
}; // ScanMatcher

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
         * @param start_vertex
         * @param visitor
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
         * @param start_vertex
         * @param visitor
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
                        double squared_distance = pose.computeSquaredDistance(center_pose_);
                        return squared_distance <= max_distance_squared_ - KT_TOLERANCE;
                } catch (...) {
                        // relocalization vertex elements missing
                        std::cout << "Unable to visit valid vertex elements!" << std::endl;
                        return false;
                }
        }
}; // NearScanVisitor

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
         * @param mapper
         * @param range_threshold
         */
        MapperGraph(Mapper *mapper, double range_threshold);

        /**
         * Adds a vertex representing the given scan to the graph
         * @param scan
         */
        Vertex<LocalizedRangeScan> *addVertex(LocalizedRangeScan *scan);

        /**
         * Creates an edge between the source scan vertex and the target scan vertex if it
         * does not already exist; otherwise return the existing edge
         * @param source_scan
         * @param target_scan
         * @param is_new_edge set to true if the edge is new
         * @return edge between source and target scan vertices
         */
        Edge<LocalizedRangeScan> *addEdge(
                LocalizedRangeScan *source_scan,
                LocalizedRangeScan *target_scan,
                bool &is_new_edge);

        /**
         * Link scan to last scan and nearby chains of scans
         * @param scan
         * @param covariance uncertainty of match
         */
        void addEdges(LocalizedRangeScan *scan, const Matrix3 &covariance);


        bool tryCloseLoop(LocalizedRangeScan *scan);

        /**
         * Optimizes scan poses
         */
        void correctPoses();

        /**
         * Find "nearby" (no further than given distance away) scans through graph links
         * @param scan
         * @param maxDistance
         */
        LocalizedRangeScanVector findNearLinkedScans(LocalizedRangeScan *scan, double max_distance);

private:
        /**
         * Adds an edge between the two scans and labels the edge with the given mean and covariance
         * @param from_scan
         * @param to_scan
         * @param mean
         * @param covariance
         */
        void linkScans(
            LocalizedRangeScan *from_scan,
            LocalizedRangeScan *to_scan,
            const Pose2 &mean,
            const Matrix3 &covariance);

        /**
         * Finds the closest scan in the vector to the given pose
         * @param scans
         * @param pose
         */
        LocalizedRangeScan *getClosestScanToPose(
            const LocalizedRangeScanVector &scans,
            const Pose2 &pose) const;

        /**
         * Tries to find a chain of scan from the given device starting at the
         * given scan index that could possibly close a loop with the given scan
         * @param scan
         * @param start_num
         * @return chain that can possibly close a loop with given scan
         */
        LocalizedRangeScanVector findPossibleLoopClosure(
            LocalizedRangeScan *scan,
            uint32_t &start_num);

        /**
         * Link the chain of scans to the given scan by finding the closest scan in the chain to the given scan
         * @param chain
         * @param scan
         * @param mean
         * @param covariance
         */
        void linkChainToScan(
                const LocalizedRangeScanVector &chain,
                LocalizedRangeScan *scan,
                const Pose2 &mean,
                const Matrix3 &covariance);

        /**
         * Find nearby chains of scans and link them to scan if response is high enough
         * @param scan
         * @param means
         * @param covariances
         */
        void linkNearChains(
            LocalizedRangeScan *scan, Pose2Vector &means,
            std::vector<Matrix3> &covariances);

        /**
         * Find chains of scans that are close to given scan
         * @param scan
         * @return chains of scans
         */
        std::vector<LocalizedRangeScanVector> findNearChains(LocalizedRangeScan *scan);

        /**
         * Compute mean of poses weighted by covariances
         * @param means
         * @param covariances
         * @return weighted mean
         */
        Pose2 computeWeightedMean(
            const Pose2Vector &means,
            const std::vector<Matrix3> &covariances) const;

        /**
         * Gets the vertex associated with the given scan
         * @param scan
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class Mapper 
{
        friend class ScanMatcher;
        friend class MapperGraph;

protected:
        bool initialized_;
        std::unique_ptr<ScanManager> scan_manager_;
        std::unique_ptr<ScanMatcher> sequential_scan_matcher_;
        std::unique_ptr<MapperGraph> graph_;
        std::unique_ptr<ScanSolver> scan_optimizer_;

        /**
         * When set to true, the mapper will use a scan matching algorithm. In most real-world situations
         * this should be set to true so that the mapper algorithm can correct for noise and errors in
         * odometry and scan data. In some simulator environments where the simulated scan and odometry
         * data are very accurate, the scan matching algorithm can produce worse results. In those cases
         * set this to false to improve results.
         * Default value is true.
         */
        bool use_scan_matching_;

        /**
         * Default value is true.
         */
        bool use_scan_barycenter_;

        /**
         * Sets the minimum travel between scans.  If a new scan's position is more than minimum_travel_distance_
         * from the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
         * new scan if it also does not meet the minimum change in heading requirement.
         * For performance reasons, generally it is a good idea to only process scans if the robot
         * has moved a reasonable amount.
         * Default value is 0.2 (meters).
         */
        double minimum_travel_distance_;

        /**
         * Sets the minimum heading change between scans. If a new scan's heading is more than minimum_travel_heading_
         * from the previous scan, the mapper will use the data from the new scan.  Otherwise, it will discard the
         * new scan if it also does not meet the minimum travel distance requirement.
         * For performance reasons, generally it is a good idea to only process scans if the robot
         * has moved a reasonable amount.
         */
        double minimum_travel_heading_;

        /**
         * Scan buffer size is the length of the scan chain stored for scan matching.
         * "scan_buffer_size_" should be set to approximately "scan_buffer_maximum_scan_distance_" / "minimum_travel_distance_".
         * The idea is to get an area approximately 20 meters long for scan matching.
         * For example, if we add scans every minimum_travel_distance_ == 0.3 meters, then "scanBufferSize"
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
         * Enable/disable loop closure.
         * Default is enabled.
         */
        bool do_loop_closing_;

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

        //////////////////////////////////////////////////////////////////////////////
        //    CorrelationParameters correlationParameters;

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

        //////////////////////////////////////////////////////////////////////////////
        //    CorrelationParameters loopCorrelationParameters;

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

        //////////////////////////////////////////////////////////////////////////////
        // ScanMatcherParameters;

        // Variance of penalty for deviating from odometry when scan-matching.
        // The penalty is a multiplier (less than 1.0) is a function of the
        // delta of the scan position being tested and the odometric pose
        double distance_variance_penalty_;
        double angle_variance_penalty_;

        // The range of angles to search during a coarse search and a finer search
        double fine_search_angle_offset_;
        double coarse_search_angle_offset_;

        // Resolution of angles to search during a coarse search
        double coarse_angle_resolution_;

        // Minimum value of the penalty multiplier so scores do not
        // become too small
        double minimum_angle_penalty_;
        double minimum_distance_penalty_;

        // Whether to increase the search space if no good matches are initially found
        bool use_response_expansion_;

        // Number of beams that must pass through a cell before it will be considered to be occupied
        // or unoccupied.  This prevents stray beams from messing up the map.
        uint32_t min_pass_through_;

        // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
        double occupancy_threshold_;

public:
        Mapper()
        : initialized_(false),
        scan_manager_(nullptr)
        {
        }

public:
        /**
         * Allocate memory needed for mapping
         * @param range_threshold
         */
        void initialize(double range_threshold)
        {
                if (initialized_ == true) {
                        return;
                }

                // create sequential scan and loop matcher

                sequential_scan_matcher_ = ScanMatcher::create(
                        this,
                        correlation_search_space_dimension_,
                        correlation_search_space_resolution_,
                        correlation_search_space_smear_deviation_,
                        range_threshold);
                assert(sequential_scan_matcher_);

                scan_manager_ = std::make_unique<ScanManager>(scan_buffer_size_, scan_buffer_maximum_scan_distance_);

                graph_ = std::make_unique<MapperGraph>(this, range_threshold);

                initialized_ = true;
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
        bool process(LocalizedRangeScan *scan, Matrix3 *covariance = nullptr);

        /** 
         * @return Create occupancy grid with the given resolution 
         */
        OccupancyGrid *getOccupancyGrid(const double &resolution)
        {
                if (!scan_manager_) {
                        return nullptr;
                }
                return OccupancyGrid::createFromScans(
                        scan_manager_->getAllScans(),
                        resolution,
                        (uint32_t)getParamMinPassThrough(),
                        (double)getParamOccupancyThreshold());
        }

public:
        void setScanSolver(std::unique_ptr<ScanSolver> scan_optimizer)
        {
                scan_optimizer_ = std::move(scan_optimizer);
        }

        ScanSolver *getScanSolver()
        {
                return scan_optimizer_.get();
        }

        MapperGraph *getGraph() const
        {
                return graph_.get();
        }

        ScanManager *getScanManager() const
        {
                return scan_manager_.get();
        }

public:
        /* Getters */
        // General Parameters
        bool getParamUseScanMatching();
        bool getParamUseScanBarycenter();
        double getParamMinimumTravelDistance();
        double getParamMinimumTravelHeading();
        int getParamScanBufferSize();
        double getParamScanBufferMaximumScanDistance();
        double getParamLinkMatchMinimumResponseFine();
        double getParamLinkScanMaximumDistance();
        double getParamLoopSearchMaximumDistance();
        bool getParamDoLoopClosing();
        int getParamLoopMatchMinimumChainSize();
        double getParamLoopMatchMaximumVarianceCoarse();
        double getParamLoopMatchMinimumResponseCoarse();
        double getParamLoopMatchMinimumResponseFine();

        // Correlation Parameters - Correlation Parameters
        double getParamCorrelationSearchSpaceDimension();
        double getParamCorrelationSearchSpaceResolution();
        double getParamCorrelationSearchSpaceSmearDeviation();

        // Correlation Parameters - Loop Closure Parameters
        double getParamLoopSearchSpaceDimension();
        double getParamLoopSearchSpaceResolution();
        double getParamLoopSearchSpaceSmearDeviation();

        // Scan Matcher Parameters
        double getParamDistanceVariancePenalty();
        double getParamAngleVariancePenalty();
        double getParamFineSearchAngleOffset();
        double getParamCoarseSearchAngleOffset();
        double getParamCoarseAngleResolution();
        double getParamMinimumAnglePenalty();
        double getParamMinimumDistancePenalty();
        bool getParamUseResponseExpansion();
        int getParamMinPassThrough();
        double getParamOccupancyThreshold();

        /* Setters */
        // General Parameters
        void setParamUseScanMatching(bool b);
        void setParamUseScanBarycenter(bool b);
        void setParamMinimumTravelDistance(double d);
        void setParamMinimumTravelHeading(double d);
        void setParamScanBufferSize(int i);
        void setParamScanBufferMaximumScanDistance(double d);
        void setParamLinkMatchMinimumResponseFine(double d);
        void setParamLinkScanMaximumDistance(double d);
        void setParamLoopSearchMaximumDistance(double d);
        void setParamDoLoopClosing(bool b);
        void setParamLoopMatchMinimumChainSize(int i);
        void setParamLoopMatchMaximumVarianceCoarse(double d);
        void setParamLoopMatchMinimumResponseCoarse(double d);
        void setParamLoopMatchMinimumResponseFine(double d);

        // Correlation Parameters - Correlation Parameters
        void setParamCorrelationSearchSpaceDimension(double d);
        void setParamCorrelationSearchSpaceResolution(double d);
        void setParamCorrelationSearchSpaceSmearDeviation(double d);

        // Correlation Parameters - Loop Closure Parameters
        void setParamLoopSearchSpaceDimension(double d);
        void setParamLoopSearchSpaceResolution(double d);
        void setParamLoopSearchSpaceSmearDeviation(double d);

        // Scan Matcher Parameters
        void setParamDistanceVariancePenalty(double d);
        void setParamAngleVariancePenalty(double d);
        void setParamFineSearchAngleOffset(double d);
        void setParamCoarseSearchAngleOffset(double d);
        void setParamCoarseAngleResolution(double d);
        void setParamMinimumAnglePenalty(double d);
        void setParamMinimumDistancePenalty(double d);
        void setParamUseResponseExpansion(bool b);
        void setParamMinPassThrough(int i);
        void setParamOccupancyThreshold(double d);

private:
        /**
         * Restrict the copy constructor
         */
        Mapper(const Mapper &);

        /**
         * Restrict the assignment operator
         */
        const Mapper &operator=(const Mapper &);

}; // Mapper


} // namespace mapper_utils

#endif // KARTO_SDK_MAPPER_HPP