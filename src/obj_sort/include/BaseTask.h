#ifndef BASE_TASK_H_
#define BASE_TASK_H_

// system includes
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>

// ROS includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/fpfh.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <pcl/features/esf.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

// Moveit specific includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// type definition 
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class BaseTask
{
public:
    /// @brief Constructor
    BaseTask(ros::NodeHandle& nh);
    
    /// @brief Destructor
    virtual ~BaseTask() = default;

    /// @brief Point cloud callback
    virtual void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
    
    /// @brief Remove all collision from motion planning
    void remove_all_collisions();
    
    /// @brief Moveit function: move arm to a given pose
    bool moveArm(geometry_msgs::Pose target_pose);
    
    /// @brief Moveit function: open/close gripper to certain width
    bool moveGripper(float width);
    
    bool homingSequence(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& place_pose);
    
    /// @brief Apply Voxel Grid filtering
    void applyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);
    
    /// @brief Normal estimation
    void findNormals(PointCPtr &in_cloud_ptr);
    
    /// @brief Segment Plane from point cloud
    void segPlane(PointCPtr &in_cloud_ptr);
    
    /// @brief Function to publish the filtered point cloud for visualization
    void pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc);
    
    /// @brief Function to calculate the orientation of the object
    tf2::Quaternion getOrientation(PointCPtr &cloud_input);
    
    /// @brief Object projection
    void projection(PointCPtr &in_cloud_ptr);
    
    /// @brief Find the Pose of Cylinder
    geometry_msgs::Point findCylPose(PointCPtr &in_cloud_ptr);
    
    /// @brief Transform point from link_8 frame to camera frame
    geometry_msgs::Point link2Cam(geometry_msgs::Point in_point);
    
    /// @brief Transform point from camera frame to link_8 frame
    geometry_msgs::Point cam2Link(geometry_msgs::Point in_point);
    
    /// @brief Apply pass through to the point cloud on x direction
    void applyPT_x(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, double &threshold);
    
    /// @brief Apply pass through to the point cloud on y direction
    void applyPT_y(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, double &threshold);
    
    /// @brief Add an object to motion planning and collision avoidance
    void addCollisionObject(std::string object_name, geometry_msgs::Point centre, 
                           geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);
    
    /// @brief Add plane to RViz
    void add_plane();
    
    /// @brief Clustering based on Euclidean distance
    void Cluster(PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &cluser_arr, double clster_tolerance);
    
    /// @brief Add collision object with simpler parameters
    void addCollision(std::string name, geometry_msgs::Pose centre, double dimensions);
    
    /// @brief Remove collision object from motion planning
    void removeCollisionObject(std::string object_name);
    
    /// @brief Pick and place operation
    bool pickAndPlace(geometry_msgs::Pose pick_pose, geometry_msgs::Pose place_pose);

    /// @brief Move arm along a Cartesian path
    bool moveArmCartesian(const std::vector<geometry_msgs::Pose>& waypoints,
                         double eef_step, double jump_threshold);
    
    /// @brief Enhanced pick and place routine with axis-by-axis movement
    bool pickAndPlaceRoutine(const geometry_msgs::Pose& ball_pose, 
                           const geometry_msgs::Pose& basket_pose,
                           double approach_height);

    /// @brief Filter point cloud based on RGB color values
    void applyColorFilter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, const std::string &color);

    /// <----------------------- Task 3 ----------------------->
    
    /// @brief Display Pointcloud in new window
    void showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);

    /// @brief Add all points to total pcl
    void addPointsToTotal(PointCPtr &g_cloud_ptr, PointCPtr &ttl_cloud);
    
    /// @brief Publish a given PCL from baseframe panda_link0
    void pubPointCloud(ros::Publisher &pc_pub, PointC &pc);

    /// @breif Normalise a given point cloud
    void normalizePointCloud(PointCPtr &cloud_ptr);
    
    /// @brief Determines if 2 point clouds are the same shape
    bool matchShapes(const PointCPtr &cloud1, const PointCPtr &cloud2);

    /// @brief determines the type of shape 
    std::string detect_type(const PointCPtr &cloud);

    /// @brief Get the centroid of a shape as a pointstamped
    geometry_msgs::PointStamped get_pcl_centrepoint(const PointCPtr &cloud);

    /// @breif Helper function to transform pointstamped and quaternion into a pose object
    geometry_msgs::Pose pointAndQuatToPose(const geometry_msgs::PointStamped &point, const tf2::Quaternion &quat);

    /// @brief Statistical Outlier removal
    void removeOutliers(PointCPtr &cloud_ptr, int meanK = 50, double stddevMulThresh = 1.0);

    /// @brief Calculates the orientation of a shape
    geometry_msgs::Quaternion task3Orientation(PointCPtr &cloud_input);

    /// @brief Gets the grasp pose from the shape centroid and orientation
    void get_grasp_pose(geometry_msgs::PoseStamped &pose_stamped);

    /// @brief Applies offset to the end effector position (needed for pickandplaceroutine)
    void applyOffsetToPose(geometry_msgs::Pose &goal_pose);

    /// @brief Pick and place function for task 3
    bool task3_pick_and_place(geometry_msgs::Pose &orig_pick_pose, geometry_msgs::Pose &orig_goal_pos);


    

protected:
    // Node handle
    ros::NodeHandle nh_;
    
    // MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
    
    // Publishers for point clouds
    ros::Publisher g_pub_rm_plane;
    ros::Publisher g_pub_rm_plane_2;
    ros::Publisher g_pub_totalcloud;
    
    // TF related variables
    tf::TransformListener g_listener_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};
    geometry_msgs::TransformStamped TranLink82Cam, TranCamera2Link8, TranBase2World;
    tf::StampedTransform stamped_transform;
    
    // Point cloud related variables
    std::string g_input_pc_frame_id_;
    pcl::PCLPointCloud2 g_pcl_pc;
    sensor_msgs::PointCloud2 g_cloud_filtered_msg;
    sensor_msgs::PointCloud2 totalcloud_msg;
    std::vector<PointCPtr> cloud_cluster_arr;
    std::vector<PointCPtr> obs_cluster_arr;
    
    // Point cloud pointers
    PointCPtr g_cloud_ptr;
    PointCPtr g_cloud_filtered_vx;
    PointCPtr g_cloud_plane;
    PointCPtr g_cloud_filtered_plane;
    PointCPtr g_cloud_projected_plane;
    PointCPtr red_filtered;
    PointCPtr blue_filtered;
    PointCPtr purp_filtered;
    PointCPtr black_filtered;
    PointCPtr goal_filtered;
    PointCPtr totalcloud_ptr;
    PointCPtr pc_object;
    PointCPtr task2_mystery_obj;
    PointCPtr task2_known_obj;
    PointCPtr total_goal_cloud;
    PointCPtr total_obs_cloud;
    
    // PCL operators
    pcl::VoxelGrid<PointT> g_vx;
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals;
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals_filtered_plane;
    pcl::PassThrough<PointT> g_pt;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg;
    pcl::ExtractIndices<PointT> g_extract_pc;
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    pcl::PointIndices::Ptr g_inliers_plane;
    pcl::ModelCoefficients::Ptr g_coeff_plane;
    pcl::ModelCoefficients::Ptr g_coeff_plane_project;
    pcl::ProjectInliers<PointT> proj;
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
    // Gripper parameters
    const double gripper_open_ = 0.08;
    const double gripper_closed_ = 0.0;
    
    // Task variables
    std::string base_frame_ = "panda_link0";
    geometry_msgs::PointStamped g_cyl_pt_msg;
    std::string obj_type;
    std::string task_2_objectType;
    double rad, degree;
    tf2::Quaternion q_object_task_1;
    bool segment_done = false;
    double square_size;
    
    // Voxel Grid and Normal parameters
    double g_vg_leaf_sz;
    int g_k_nn;
};

#endif // BASE_TASK_H_ 
