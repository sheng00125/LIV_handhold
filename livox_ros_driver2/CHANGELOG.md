# Changelog

All notable changes to this project will be documented in this file.

## [1.2.4]
### Fixed
- Optimize framing performance

## [1.2.3]
### Fixed
- Optimize framing logic and reduce CPU usage
- Fixed some known issues

## [1.2.1]
### Fixed
- Fix offset time error regarding CustomMsg format message publishment.

## [1.2.0]
### Added
- Revise the frame segmentation logic.
- (Notice!!!) Add Timestamp to each point in Livox pointcloud2 (PointXYZRTLT) format. The PointXYZRTL format has been updated to PointXYZRTLT format. Compatibility needs to be considered.
### Fixed
- Improve support for gPTP and GPS synchronizations.

--- 
## [1.1.3]
### Fixed
- Improve performance when running in ROS2 Humble.

--- 
## [1.1.2]
### Changed
- Change publish frequency range to [0.5Hz, 10 Hz].
### Fixed
- Fix a high CPU-usage problem.

--- 
## [1.1.1]
### Added
- Offer valid line-number info in the point cloud data of MID-360 Lidar.
- Enable IMU by default.
### Changed
- Update the README slightly.

--- 
## [1.0.0]
### Added
- Support Mid-360 Lidar.
- Support for Ubuntu 22.04 ROS2 humble.
- Support multi-topic fuction, the suffix of the topic name corresponds to the ip address of each Lidar. 
### Changed
- Remove the embedded SDK.
- Constraint: Livox ROS Driver 2 for ROS2 does not support message passing with PCL native data types.
### Fixed
- Fix IMU packet loss.
- Fix some conflicts with livox ros driver.
- Fixed HAP Lidar publishing PointCloud2 and CustomMsg format point clouds with no line number.
