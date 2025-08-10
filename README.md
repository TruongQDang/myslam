# LiDAR-based SLAM
This is a minimal LiDAR SLAM based on SLAM Toolbox and Karto SLAM. It is only capable of processing scans from a single LiDAR.

## Project Structure
The project is organized as follows:

![](docs/architecture.png)

It consists of a ROS2 wrapper, a standalone SLAM library, and CeresSolver as back-end optmization.

## SLAM
The SLAM library is organized as follows:

![](docs/slam_architecture.png)

It consists of several main classes:
- `Mapper` acts as the main interface to accept inputs and return outputs of the system.
- `ScanManager` handles keeping track of incoming laser scans.
- `ScanMatcher` handles scan matching, crucial for correcting odometry as well as loop closure validation.
- `MapperGraph` handle graph construction and loop search.
- `OccupancyGrid` returns occupancy grid map built from a list of laser scans and robot's poses.
- `ScanSolver` is a interface class with virtual functions to allow for flexibility in choosing whichever optimization library that users may wish.
