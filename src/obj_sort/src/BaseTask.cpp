#include "BaseTask.h"

BaseTask::BaseTask(ros::NodeHandle& nh) : 
    g_cloud_ptr(new PointC),
    g_cloud_filtered_vx(new PointC),
    g_cloud_plane(new PointC),
    g_cloud_filtered_plane(new PointC),
    g_tree_ptr(new pcl::search::KdTree<PointT>),
    g_cloud_normals(new pcl::PointCloud<pcl::Normal>),
    g_cloud_normals_filtered_plane(new pcl::PointCloud<pcl::Normal>),
    g_inliers_plane(new pcl::PointIndices),
    g_coeff_plane(new pcl::ModelCoefficients),
    g_coeff_plane_project(new pcl::ModelCoefficients()),
    g_cloud_projected_plane(new PointC),
    totalcloud_ptr(new PointC),
    red_filtered(new PointC),
    blue_filtered(new PointC),
    purp_filtered(new PointC),
    black_filtered(new PointC),
    goal_filtered(new PointC),
    pc_object(new PointC),
    task2_mystery_obj(new PointC),
    task2_known_obj(new PointC),
    total_goal_cloud(new PointC),
    total_obs_cloud(new PointC)
{
    nh_ = nh;
    
    // Reduce verbosity of ROS logging - only show WARN and above for most components
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    // Set up PCL parameters
    g_vg_leaf_sz = 0.001f;
    g_k_nn = 50;
    
    // Define the publishers for visualization
    g_pub_rm_plane = nh_.advertise<sensor_msgs::PointCloud2>("/1", 1, true);
    g_pub_rm_plane_2 = nh_.advertise<sensor_msgs::PointCloud2>("/2", 1, true);
    g_pub_totalcloud = nh_.advertise<sensor_msgs::PointCloud2>("/3", 1, true);
    
    // Transformation from camera to panda link 8
    try {
        TranCamera2Link8 = tfBuffer.lookupTransform("panda_link8", "rs200_camera", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        // Keep this warning as it's critical for troubleshooting TF issues
        ROS_WARN("%s", ex.what());
    }

    // Transformation from panda link_8 to camera  
    try {
        TranLink82Cam = tfBuffer.lookupTransform("rs200_camera", "panda_link8", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        // Keep this warning as it's critical for troubleshooting TF issues
        ROS_WARN("%s", ex.what());
    }

    TranCamera2Link8.transform.translation.x = -TranLink82Cam.transform.translation.x;
    TranCamera2Link8.transform.translation.y = -TranLink82Cam.transform.translation.y;
    TranCamera2Link8.transform.translation.z = -TranLink82Cam.transform.translation.z;
}

void BaseTask::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
    // This is a base implementation that should be overridden by task-specific classes
    // It contains common functionality that all tasks might need
    
    // Extract input point cloud info
    g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
    pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

    // Downsampling the point cloud
    applyVX(g_cloud_ptr, g_cloud_filtered_vx);

    // Apply color filtering (using blue as default, derived classes should specify color)
    PointCPtr cloud_filtered_color(new PointC);
    applyColorFilter(g_cloud_filtered_vx, cloud_filtered_color, "blue");
    
    // Update the filtered cloud for further processing
    *g_cloud_filtered_vx = *cloud_filtered_color;

    // Segment the plane
    findNormals(g_cloud_filtered_vx);
    segPlane(g_cloud_filtered_vx);
}

bool BaseTask::moveArm(geometry_msgs::Pose target_pose) {
    // Setup the target pose
    arm_group_.setPoseTarget(target_pose);

    // Create a movement plan for the arm
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group_.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute the planned path
    arm_group_.move();

    return success;
}

bool BaseTask::moveGripper(float width) {
    // Safety checks in case width exceeds safe values
    if (width > gripper_open_)
        width = gripper_open_;
    if (width < gripper_closed_)
        width = gripper_closed_;

    // Calculate the joint targets as half each of the requested distance
    double eachJoint = width / 2.0;

    // Create a vector to hold the joint target for each joint
    std::vector<double> gripperJointTargets(2);
    gripperJointTargets[0] = eachJoint;
    gripperJointTargets[1] = eachJoint;

    // Apply the joint target
    hand_group_.setJointValueTarget(gripperJointTargets);

    // Move the robot hand
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (hand_group_.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Move the gripper joints
    hand_group_.move();

    return success;
}

/**
 * @brief Moves the robot arm following a specified Cartesian path
 * 
 * This function plans and executes a motion that passes through all specified waypoints:
 * 1. Takes a vector of poses representing the desired path
 * 2. Computes a Cartesian path through these waypoints
 * 3. Executes the calculated trajectory if valid
 * 
 * @param waypoints Vector of poses representing the desired path
 * @param eef_step Max distance between end-effector positions in Cartesian space
 * @param jump_threshold Max allowed joint value jump between consecutive points
 * @return true if movement was successful, false if planning or execution failed
 */
bool BaseTask::moveArmCartesian(const std::vector<geometry_msgs::Pose> &waypoints, 
                               double eef_step, double jump_threshold) {
    const int MAX_RETRIES = 3;
    const double PLANNING_TIME = 10.0;
    const double WAYPOINT_REACHED_TOLERANCE = 0.02; // 2cm tolerance for considering a waypoint reached
    
    // Make a copy of the waypoints that we can modify
    std::vector<geometry_msgs::Pose> remaining_waypoints = waypoints;
    int total_waypoints = waypoints.size();
    int current_waypoint_index = 0;
    
    while (!remaining_waypoints.empty() && current_waypoint_index < total_waypoints) {
        ROS_INFO("Planning Cartesian path through %zu remaining waypoints", 
                 remaining_waypoints.size());
        
        // Set execution timeout and tolerances
        arm_group_.setPlanningTime(PLANNING_TIME);
        arm_group_.setGoalPositionTolerance(0.01);
        arm_group_.setGoalOrientationTolerance(0.01);
        
        // Compute the Cartesian path for remaining waypoints
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = arm_group_.computeCartesianPath(remaining_waypoints, 
                                                         eef_step,
                                                         jump_threshold,
                                                         trajectory);
        
        ROS_INFO("Visualising Cartesian path (%.2f%% achieved)", fraction * 100.0);
        
        if (fraction > 0.0) {
            // Create a plan from the calculated trajectory
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            
            // Ensure proper timing of the trajectory
            robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
            rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), cartesian_plan.trajectory_);            
            
            // Execute the plan
            ROS_INFO("Executing Cartesian path plan");
            bool execution_success = (arm_group_.execute(cartesian_plan) == 
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (execution_success) {
                ROS_INFO("Cartesian movement succeeded");
                return true;
            }
            
            // If execution failed, check which waypoints we've reached
            ROS_WARN("Execution failed, checking progress...");
            
            // Get current robot position
            geometry_msgs::Pose current_pose = arm_group_.getCurrentPose().pose;
            
            // Check which waypoints we've reached
            bool made_progress = false;
            while (current_waypoint_index < total_waypoints && !remaining_waypoints.empty()) {
                // Calculate distance to the next waypoint
                double dx = current_pose.position.x - remaining_waypoints[0].position.x;
                double dy = current_pose.position.y - remaining_waypoints[0].position.y;
                double dz = current_pose.position.z - remaining_waypoints[0].position.z;
                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                if (distance <= WAYPOINT_REACHED_TOLERANCE) {
                    // We've reached this waypoint, remove it
                    ROS_INFO("Reached waypoint %d", current_waypoint_index);
                    remaining_waypoints.erase(remaining_waypoints.begin());
                    current_waypoint_index++;
                    made_progress = true;
                } else {
                    // We haven't reached this waypoint yet
                    break;
                }
            }
            
            if (!made_progress) {
                // We didn't reach any new waypoints, retry with a different approach
                ROS_WARN("No progress made, trying direct move to next waypoint");
                
                // Try a direct move to the next waypoint
                arm_group_.setPoseTarget(remaining_waypoints[0]);
                moveit::planning_interface::MoveGroupInterface::Plan direct_plan;
                bool plan_success = (arm_group_.plan(direct_plan) == 
                                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
                
                if (plan_success) {
                    bool move_success = (arm_group_.execute(direct_plan) == 
                                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    
                    if (move_success) {
                        // We reached the next waypoint with direct move
                        ROS_INFO("Direct move to waypoint %d succeeded", current_waypoint_index);
                        remaining_waypoints.erase(remaining_waypoints.begin());
                        current_waypoint_index++;
                    } else {
                        // Direct move failed too, skip this waypoint
                        ROS_WARN("Direct move failed, skipping waypoint %d", current_waypoint_index);
                        remaining_waypoints.erase(remaining_waypoints.begin());
                        current_waypoint_index++;
                    }
                } else {
                    // Planning failed, skip this waypoint
                    ROS_WARN("Direct move planning failed, skipping waypoint %d", current_waypoint_index);
                    remaining_waypoints.erase(remaining_waypoints.begin());
                    current_waypoint_index++;
                }
            }
            
            // Brief pause before continuing
            ros::Duration(0.5).sleep();
            
        } else {
            // Failed to compute path, try direct move to next waypoint
            ROS_WARN("Failed to compute Cartesian path, trying direct move");
            
            arm_group_.setPoseTarget(remaining_waypoints[0]);
            moveit::planning_interface::MoveGroupInterface::Plan direct_plan;
            bool plan_success = (arm_group_.plan(direct_plan) == 
                                moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (plan_success) {
                bool move_success = (arm_group_.execute(direct_plan) == 
                                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
                
                if (move_success) {
                    // We reached the next waypoint with direct move
                    ROS_INFO("Direct move to waypoint %d succeeded", current_waypoint_index);
                    remaining_waypoints.erase(remaining_waypoints.begin());
                    current_waypoint_index++;
                } else {
                    // Direct move failed, skip this waypoint
                    ROS_WARN("Direct move failed, skipping waypoint %d", current_waypoint_index);
                    remaining_waypoints.erase(remaining_waypoints.begin());
                    current_waypoint_index++;
                }
            } else {
                // Planning failed, skip this waypoint
                ROS_WARN("Direct move planning failed, skipping waypoint %d", current_waypoint_index);
                remaining_waypoints.erase(remaining_waypoints.begin());
                current_waypoint_index++;
            }
            
            ros::Duration(0.5).sleep();
        }
        
        // Check if we've tried too many times
        if (current_waypoint_index >= total_waypoints * 2) {
            ROS_ERROR("Too many retries, giving up");
            return false;
        }
    }
    
    // If we've reached all waypoints, return success
    if (remaining_waypoints.empty()) {
        ROS_INFO("Successfully reached all waypoints");
        return true;
    }
    
    ROS_ERROR("Failed to execute complete Cartesian movement");
    return false;
}

void BaseTask::remove_all_collisions() {
    // Create a collision object message, and a vector of these messages
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> object_vector;

    collision_object.operation = collision_object.REMOVE;
    object_vector.push_back(collision_object);
    planning_scene_interface_.applyCollisionObjects(object_vector);
}

void BaseTask::applyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr) {
    g_vx.setInputCloud(in_cloud_ptr);
    g_vx.setLeafSize(g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
    g_vx.filter(*out_cloud_ptr);
}

void BaseTask::findNormals(PointCPtr &in_cloud_ptr) {
    // Estimate point normals
    g_ne.setInputCloud(in_cloud_ptr);
    g_ne.setSearchMethod(g_tree_ptr);
    g_ne.setKSearch(g_k_nn);
    g_ne.compute(*g_cloud_normals);
}

void BaseTask::segPlane(PointCPtr &in_cloud_ptr) {
    // Create the segmentation object for the planar model
    // and set all the params
    g_seg.setOptimizeCoefficients(true);
    g_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    g_seg.setNormalDistanceWeight(0.0001);
    g_seg.setMethodType(pcl::SAC_RANSAC);
    g_seg.setMaxIterations(1000);
    g_seg.setDistanceThreshold(0.03);
    g_seg.setInputCloud(in_cloud_ptr);
    g_seg.setInputNormals(g_cloud_normals);
    
    // Obtain the plane inliers and coefficients
    g_seg.segment(*g_inliers_plane, *g_coeff_plane);

    // Extract the planar inliers from the input cloud
    g_extract_pc.setInputCloud(in_cloud_ptr);
    g_extract_pc.setIndices(g_inliers_plane);
    g_extract_pc.setNegative(false);

    // Write the planar inliers to disk
    g_extract_pc.filter(*g_cloud_plane);

    // Remove the planar inliers, extract the rest
    g_extract_pc.setNegative(true);
    g_extract_pc.filter(*g_cloud_filtered_plane);
    g_extract_normals.setNegative(true);
    g_extract_normals.setInputCloud(g_cloud_normals);
    g_extract_normals.setIndices(g_inliers_plane);
    g_extract_normals.filter(*g_cloud_normals_filtered_plane);
}

void BaseTask::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc) {
    // Publish the data
    pcl::toROSMsg(pc, g_cloud_filtered_msg);
    pc_pub.publish(g_cloud_filtered_msg);
}

tf2::Quaternion BaseTask::getOrientation(PointCPtr &cloud_input) {
    // Calculate the max and min coordinate in the filtered point cloud
    PointT min_point, max_point;
    pcl::getMinMax3D(*cloud_input, min_point, max_point);

    // Define the orientation and initialize
    tf2::Quaternion q(-1, 0, 0, 0), q_rota, q_2, q_3;
    q_2.setRPY(0, 0, M_PI / 4);
    q_3.setRPY(0, 0, -M_PI / 4);
    geometry_msgs::Point point_1;
    double max_y = -100;
    int count = 0;
    double num_points = static_cast<double>(cloud_input->size());

    if (obj_type == "nought") {
        for (int i = 0; i < num_points; ++i) {
            if (abs(cloud_input->points[i].y - max_point.y) < 0.008) {
                point_1.x += cloud_input->points[i].x;
                point_1.y += cloud_input->points[i].y;
                count += 1;
            }
        }

        // Calculate the target point for the orientation calculation
        point_1.x = point_1.x / count;
        point_1.y = point_1.y / count;

        rad = atan2(point_1.y, point_1.x);
        rad = abs(fmod(rad, M_PI_2));
        degree = rad / M_PI * 180;
        // For the nought, a 45 degrees offset need to be consider
        degree = degree - 45;
        // Ensure the orientation is between 0 and pi/2
        q_rota.setRPY(0, 0, abs(fmod(rad + M_PI_4, M_PI_2)));
        // 45 degrees offset of the end effector
        q_rota = q_rota * q_2;
        // Ensure the orientation is between 0 and pi/2
        rad = abs(fmod(rad + M_PI_4, M_PI_2));
    } else if (obj_type == "cross") {
        for (int i = 1; i < num_points; ++i) {
            if (abs(cloud_input->points[i].y - max_point.y) < 0.005) {
                pcl::PointXYZ point_pre(cloud_input->points[i - 1].x, 
                                        cloud_input->points[i - 1].y, 
                                        cloud_input->points[i - 1].z);

                pcl::PointXYZ point_after(cloud_input->points[i].x, 
                                          cloud_input->points[i].y, 
                                          cloud_input->points[i].z);

                if (abs(pcl::euclideanDistance(point_pre, point_after)) < 0.05) {
                    point_1.x += cloud_input->points[i].x;
                    point_1.y += cloud_input->points[i].y;
                    count += 1;
                }
            }
        }

        point_1.x = point_1.x / count;
        point_1.y = point_1.y / count;

        rad = atan2(point_1.y, point_1.x);
        rad = abs(fmod(rad, M_PI_2));
        degree = rad / M_PI * 180;
        q_rota.setRPY(0, 0, (fmod(rad + M_PI_2, M_PI_2)) - M_PI_2);
        q_rota = q_rota * q_2;
        rad = abs(fmod(rad, M_PI_2));
    }

    // Flip the end effector
    q = q * q_rota;

    return q;
}

void BaseTask::projection(PointCPtr &in_cloud_ptr) {
    g_coeff_plane_project->values.resize(4);
    g_coeff_plane_project->values[0] = 0;    // plane normal x
    g_coeff_plane_project->values[1] = 0;    // plane normal y
    g_coeff_plane_project->values[2] = 1;    // plane normal z
    g_coeff_plane_project->values[3] = -0.6; // plane distance from origin

    proj.setInputCloud(in_cloud_ptr);
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setModelCoefficients(g_coeff_plane_project);
    proj.filter(*g_cloud_projected_plane);
}

geometry_msgs::Point BaseTask::findCylPose(PointCPtr &in_cloud_ptr) {
    Eigen::Vector4f centroid_in;
    pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);

    g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
    g_cyl_pt_msg.header.stamp = ros::Time(0);
    g_cyl_pt_msg.point.x = centroid_in[0];
    g_cyl_pt_msg.point.y = centroid_in[1];
    g_cyl_pt_msg.point.z = centroid_in[2];

    // Transform the point to new frame
    geometry_msgs::PointStamped g_cyl_pt_msg_out;
    try {
        g_listener_.transformPoint("panda_link0",
                                g_cyl_pt_msg,
                                g_cyl_pt_msg_out);
    } catch (tf::TransformException &ex) {
        // ROS_ERROR("Received a transformation exception: %s", ex.what());
    }

    geometry_msgs::Point result;
    result.x = g_cyl_pt_msg_out.point.x;
    result.y = g_cyl_pt_msg_out.point.y;
    result.z = g_cyl_pt_msg_out.point.z;

    return result;
}

geometry_msgs::Point BaseTask::link2Cam(geometry_msgs::Point in_point) {
    // Initialize
    geometry_msgs::PointStamped point_link, point_cam;
    tf2::Quaternion q_0(0, 0, 0, 1);
    TranLink82Cam.transform.rotation = tf2::toMsg(q_0);
    point_link.point = in_point;
    tf2::doTransform(point_link, point_cam, TranLink82Cam);

    return point_cam.point;
}

geometry_msgs::Point BaseTask::cam2Link(geometry_msgs::Point in_point) {
    // Initialize
    geometry_msgs::PointStamped point_link, point_cam;
    tf2::Quaternion q_0(0, 0, 0, 1);
    TranCamera2Link8.transform.rotation = tf2::toMsg(q_0);
    point_cam.point = in_point;
    tf2::doTransform(point_cam, point_link, TranCamera2Link8);

    return point_link.point;
}

void BaseTask::applyPT_x(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, double &threshold) {
    double g_pt_thrs = threshold;
    g_pt.setInputCloud(in_cloud_ptr);
    g_pt.setFilterFieldName("x");
    g_pt.setFilterLimits(-g_pt_thrs, g_pt_thrs);
    g_pt.filter(*out_cloud_ptr);
}

void BaseTask::applyPT_y(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, double &threshold) {
    double g_pt_thrs = threshold;
    g_pt.setInputCloud(in_cloud_ptr);
    g_pt.setFilterFieldName("y");
    g_pt.setFilterLimits(-g_pt_thrs, g_pt_thrs);
    g_pt.filter(*out_cloud_ptr);
}

void BaseTask::addCollisionObject(std::string object_name, 
                                 geometry_msgs::Point centre,
                                 geometry_msgs::Vector3 dimensions,
                                 geometry_msgs::Quaternion orientation) {
    // Create a collision object message, and a vector of these messages
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> object_vector;

    // Input header information
    collision_object.id = object_name;
    collision_object.header.frame_id = base_frame_;

    // Define the primitive and its dimensions
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = dimensions.x;
    collision_object.primitives[0].dimensions[1] = dimensions.y;
    collision_object.primitives[0].dimensions[2] = dimensions.z;

    // Define the pose of the collision object
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = centre.x;
    collision_object.primitive_poses[0].position.y = centre.y;
    collision_object.primitive_poses[0].position.z = centre.z;
    collision_object.primitive_poses[0].orientation = orientation;

    // Define that we will be adding this collision object
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the vector, then apply to planning scene
    object_vector.push_back(collision_object);
    planning_scene_interface_.applyCollisionObjects(object_vector);
}

void BaseTask::add_plane() {
    // Add bottom plane (i.e., ground)
    geometry_msgs::Point ground_centre; // Ground centre
    ground_centre.x = 0;
    ground_centre.y = 0;
    ground_centre.z = 0.01; // Derive from cube center position and dimension
    
    geometry_msgs::Vector3 ground_dimension; // Ground dimension
    ground_dimension.x = 5;
    ground_dimension.y = 5;
    ground_dimension.z = 0.02;
    
    geometry_msgs::Quaternion ground_orientation; // Ground orientation
    ground_orientation.w = 1;
    ground_orientation.x = 0;
    ground_orientation.y = 0;
    ground_orientation.z = 0;
    
    addCollisionObject("plane", ground_centre, ground_dimension, ground_orientation);
}

void 
BaseTask::Cluster(PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &cluser_arr, double clster_tolerance) {
    ROS_INFO("Start clustering");

    pcl::search::Octree<PointT>::Ptr octree_ptr(new pcl::search::Octree<PointT>(0.005));
    octree_ptr->setInputCloud(in_cloud_ptr);
    
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices.clear();

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clster_tolerance);
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(octree_ptr);
    ec.setInputCloud(in_cloud_ptr);
    ec.extract(cluster_indices);

    std::cout << "##################################" << std::endl;
    std::cout << "Number of clusters is " << cluster_indices.size() << std::endl;
    std::cout << "##################################" << std::endl;

    cluser_arr.clear();

    for (const auto &cluster : cluster_indices) {
        PointCPtr cloud_cluster(new PointC);
        for (const auto &idx : cluster.indices) {
            cloud_cluster->push_back((*in_cloud_ptr)[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cluser_arr.push_back(cloud_cluster);
    }
}

void BaseTask::addCollision(std::string name, geometry_msgs::Pose centre, double dimensions) {
    // Create a collision object message, and a vector of these messages
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> object_vector;

    // Input header information
    collision_object.id = name;
    collision_object.header.frame_id = base_frame_;

    // Define the primitive and its dimensions
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    
    if (dimensions == 0.02) {
        collision_object.primitives[0].dimensions[0] = 0.02*5;
        collision_object.primitives[0].dimensions[1] = 0.02*5;
        collision_object.primitives[0].dimensions[2] = 0.02;
    } else if (dimensions == 0.03) {
        collision_object.primitives[0].dimensions[0] = 0.03*5;
        collision_object.primitives[0].dimensions[1] = 0.03*5;
        collision_object.primitives[0].dimensions[2] = 0.03;
    } else if (dimensions == 0.04) {
        collision_object.primitives[0].dimensions[0] = 0.04*5;
        collision_object.primitives[0].dimensions[1] = 0.04*5;
        collision_object.primitives[0].dimensions[2] = 0.04;
    }
    
    // Define the pose of the collision object
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = centre.position.x;
    collision_object.primitive_poses[0].position.y = centre.position.y;
    collision_object.primitive_poses[0].position.z = 0.04;
    collision_object.primitive_poses[0].orientation = centre.orientation;

    // Define that we will be adding this collision object
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the vector, then apply to planning scene
    object_vector.push_back(collision_object);
    planning_scene_interface_.applyCollisionObjects(object_vector);
}

void BaseTask::removeCollisionObject(std::string object_name) {
    // Create a collision object message, and a vector of these messages
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> object_vector;
    
    // Input header information
    collision_object.id = object_name;
    collision_object.header.frame_id = base_frame_;

    // Define that we will be removing this collision object 
    collision_object.operation = collision_object.REMOVE;

    object_vector.push_back(collision_object);
    planning_scene_interface_.applyCollisionObjects(object_vector);
}

bool BaseTask::pickAndPlaceRoutine(const geometry_msgs::Pose& ball_pose,
                                 const geometry_msgs::Pose& basket_pose,
                                 double approach_height) {
    // Create waypoint vectors for each movement sequence
    std::vector<geometry_msgs::Pose> waypoints;

    // Create local copies of poses that we can modify
    geometry_msgs::Pose pick_pose = ball_pose;
    geometry_msgs::Pose place_pose = basket_pose;
    place_pose.position.z += 0.05;

    // current position of the robot
    geometry_msgs::Pose current_pose;

    // Set movement parameters
    const double eef_step = 0.01;  // 1cm steps
    const double approach_offset = approach_height;  // Height above objects
    
    // Open gripper at start
    if (!moveGripper(gripper_open_)) {
        return false;
    }
    
    // 1. Move above ball
    geometry_msgs::Pose above_ball = pick_pose;

    above_ball.position.z += approach_offset;
    geometry_msgs::Pose above_ball_single_axis = above_ball;
    current_pose = arm_group_.getCurrentPose().pose;

    if (abs(above_ball.position.x - current_pose.position.x) < abs(above_ball.position.y - current_pose.position.y)) {
        above_ball_single_axis.position.y = current_pose.position.y;
    } else {
        above_ball_single_axis.position.x = current_pose.position.x;
    }

    waypoints.clear();
    waypoints.push_back(above_ball_single_axis);
    waypoints.push_back(above_ball);
    waypoints.push_back(pick_pose);

    if (!moveArmCartesian(waypoints, eef_step, 0.0)) {
        return false;
    }
    
    // 2. Close gripper to grasp ball
    if (!moveGripper(gripper_closed_)) {
        return false;
    }
    
    // 3. Move above basket
    geometry_msgs::Pose above_basket = place_pose;
    above_basket.position.z += approach_offset;

    geometry_msgs::Pose above_basket_single_axis = above_basket;
    current_pose = arm_group_.getCurrentPose().pose;

    if (abs(above_basket.position.x - current_pose.position.x) < abs(above_basket.position.y - current_pose.position.y)) {
        above_basket_single_axis.position.y = current_pose.position.y;
    } else {
        above_basket_single_axis.position.x = current_pose.position.x;
    }


    waypoints.clear();
    waypoints.push_back(above_basket_single_axis);
    waypoints.push_back(above_basket);
    waypoints.push_back(place_pose);
    if (!moveArmCartesian(waypoints, eef_step, 0.0)) {
        return false;
    }
    
    // 4. Open gripper to release ball - Modified for more robustness
    const int MAX_RETRIES = 3;
    const double GRIPPER_TOLERANCE = 0.001;  // 1mm tolerance
    
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        if (!moveGripper(gripper_open_)) {
            continue;
        }
        
        // Longer delay for gripper movement
        ros::Duration(1.0).sleep();
        
        // Verify gripper state
        std::vector<double> current_positions = hand_group_.getCurrentJointValues();
        double current_width = current_positions[0] + current_positions[1];
        
        if (std::abs(current_width - gripper_open_) <= GRIPPER_TOLERANCE) {
            return true;
        }
    }
    
    return false;
}

void BaseTask::applyColorFilter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, const std::string &color) {
    // Create a new filtered cloud
    out_cloud_ptr->clear();
    out_cloud_ptr->header = in_cloud_ptr->header;
    
    // Define color thresholds (RGB values normalized to 0-1)
    float r_target, g_target, b_target;
    float r_threshold, g_threshold, b_threshold;
    
    if (color == "blue") {
        // Blue objects have low R, low G, high B
        r_target = 0.1f; g_target = 0.1f; b_target = 0.8f;
        // Blue is more sensitive to R and G contamination, so tighter thresholds
        r_threshold = 0.15f; g_threshold = 0.15f; b_threshold = 0.25f;
    } else if (color == "red") {
        // Red objects have high R, low G, low B
        r_target = 0.9f; g_target = 0.1f; b_target = 0.1f;
        // Red can have G and B contamination but R must be high
        r_threshold = 0.25f; g_threshold = 0.15f; b_threshold = 0.15f;
    } else if (color == "purple") {
        // Purple objects have high R, low G, high B
        r_target = 0.8f; g_target = 0.1f; b_target = 0.8f;
        // Purple is sensitive to G contamination but more tolerant of R/B variation
        r_threshold = 0.25f; g_threshold = 0.15f; b_threshold = 0.25f;
    } else if (color == "black") {
        // Black objects have low R, low G, low B
        r_target = 0.0f; g_target = 0.0f; b_target = 0.0f;
        // Purple is sensitive to G contamination but more tolerant of R/B variation
        r_threshold = 0.1f; g_threshold = 0.1f; b_threshold = 0.1f;
    } else if (color == "goal") {
        // Red objects have high R, low G, low B
        r_target = 0.5f; g_target = 0.2f; b_target = 0.2f;
        // Red can have G and B contamination but R must be high
        r_threshold = 0.05f; g_threshold = 0.05f; b_threshold = 0.05f;
    } else {
        *out_cloud_ptr = *in_cloud_ptr;
        return;
    }
    
    // Minimum color intensity to consider (helps filter out dark/black areas)
    const float min_intensity = 0.2f;
    
    // Use HSV-like logic to handle lighting variations better
    for (const auto& point : in_cloud_ptr->points) {
        // Convert RGB values from 0-255 to 0-1 range
        float r = static_cast<float>(point.r) / 255.0f;
        float g = static_cast<float>(point.g) / 255.0f;
        float b = static_cast<float>(point.b) / 255.0f;
        
        // Calculate total intensity
        float intensity = r + g + b;
        
        // Skip points that are too dark
        if (intensity < min_intensity) continue;
        
        // For blue objects
        if (color == "blue") {
            // Blue component should be higher than others
            if (b > r + 0.1f && b > g + 0.1f && 
                std::abs(r - r_target) < r_threshold &&
                std::abs(g - g_target) < g_threshold &&
                std::abs(b - b_target) < b_threshold) {
                out_cloud_ptr->points.push_back(point);
            }
        } 
        // For red objects
        else if (color == "red") {
            // Red component should be higher than others
            if (r > g + 0.1f && r > b + 0.1f && 
                std::abs(r - r_target) < r_threshold &&
                std::abs(g - g_target) < g_threshold &&
                std::abs(b - b_target) < b_threshold) {
                out_cloud_ptr->points.push_back(point);
            }
        } 
        // For purple objects
        else if (color == "purple") {
            // Both red and blue should be higher than green
            if (std::abs(r - r_target) < r_threshold &&
                std::abs(g - g_target) < g_threshold &&
                std::abs(b - b_target) < b_threshold) {
                out_cloud_ptr->points.push_back(point);
            }
        }
        // For Black Objects
        else if (color == "black") {
            // Both red and blue should be higher than green
            if (std::abs(r - r_target) < r_threshold &&
                std::abs(g - g_target) < g_threshold &&
                std::abs(b - b_target) < b_threshold) {
                out_cloud_ptr->points.push_back(point);
            }
        }
        // For Goal objects
        else if (color == "goal") {
            // Red component should be higher than others
            if (r > g + 0.1f && r > b + 0.1f && 
                std::abs(r - r_target) < r_threshold &&
                std::abs(g - g_target) < g_threshold &&
                std::abs(b - b_target) < b_threshold) {
                out_cloud_ptr->points.push_back(point);
            }
        } 
    }
} 

void
BaseTask::showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud) {
    /*
    * Creates point cloud viewer window
    */
  pcl::visualization::CloudViewer viewer("Cluster Viewer");
  viewer.showCloud(input_cloud);
}

void 
BaseTask::addPointsToTotal(PointCPtr &g_cloud_ptr, PointCPtr &ttl_cloud) {
    /* 
     * Arguments:
     *   g_cloud_ptr: Pointer to the point cloud containing new points, expressed in the end effector's frame.
     *   end_effector_transform: Transformation from the end effector frame to the global frame.
     *
     * Returns: None. The function transforms the incoming point cloud into the global frame and appends its points to totalcloud_ptr.
     */
    
    PointCPtr transformed_cloud(new PointC);
    Eigen::Affine3d transform_d = tf2::transformToEigen(TranBase2World);
    Eigen::Affine3f transform_f = transform_d.cast<float>();
    pcl::transformPointCloud(*g_cloud_ptr, *transformed_cloud, transform_f);

    *ttl_cloud += *transformed_cloud;
}

void 
BaseTask::pubPointCloud(ros::Publisher &pc_pub, PointC &pc) {
    /*
    * Arguments: 
    *   ros::Publisher pc_pub: nh publisher element
    *   PointC pc: pointcloud to be published
    */
    pcl::toROSMsg(pc, totalcloud_msg);
    totalcloud_msg.header.frame_id = base_frame_;
    pc_pub.publish(totalcloud_msg);
}

void 
BaseTask::normalizePointCloud(PointCPtr &cloud_ptr) {
    /* 
        * Arguments:
        *   cloud_ptr: Pointer to the point cloud to be normalized.
        *
        * Returns: None. The function first downsamples the point cloud using a voxel grid filter,
        *          then centers it at the origin and scales it so that the maximum distance from the origin is 1.
        */
    
    // Downsample the point cloud
    PointCPtr downsampled(new PointC);
    pcl::VoxelGrid<PointT> voxel;
    // Set a leaf size; adjust as needed to balance detail vs. point reduction
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.setInputCloud(cloud_ptr);
    voxel.filter(*downsampled);
    
    // Replace the input cloud with the downsampled version
    *cloud_ptr = *downsampled;
    
    // Compute the centroid of the downsampled cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_ptr, centroid);
    
    // Center the cloud by subtracting the centroid
    for (auto &point : cloud_ptr->points) {
        point.x -= centroid[0];
        point.y -= centroid[1];
        point.z -= centroid[2];
    }
    
    // Compute the maximum distance from the origin
    float max_dist = 0.0f;
    for (const auto &point : cloud_ptr->points) {
        float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (dist > max_dist) {
            max_dist = dist;
        }
    }
    
    // Scale the cloud so that the maximum distance is 1
    if (max_dist > 0.0f) {
        for (auto &point : cloud_ptr->points) {
            point.x /= max_dist;
            point.y /= max_dist;
            point.z /= max_dist;
        }
    }
}

bool 
BaseTask::matchShapes(const PointCPtr &cloud1, const PointCPtr &cloud2) {
    /* 
     * Arguments:
     *   cloud1: Pointer to the first normalized point cloud.
     *   cloud2: Pointer to the second normalized point cloud.
     *
     * Returns:
     *   true if the ESF descriptors of the two clouds are similar enough (indicating that the shapes match),
     *   false otherwise.
     *
     * This function computes an ESF descriptor (a 640-bin histogram) for each point cloud using PCL's ESFEstimation,
     * then calculates the Euclidean distance between the two histograms. A match is declared if the distance is below a set threshold.
     */
    
    pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf;
    
    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor1(new pcl::PointCloud<pcl::ESFSignature640>());
    esf.setInputCloud(cloud1);
    esf.compute(*descriptor1);
    
    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor2(new pcl::PointCloud<pcl::ESFSignature640>());
    esf.setInputCloud(cloud2);
    esf.compute(*descriptor2);
    
    if (descriptor1->points.empty() || descriptor2->points.empty()) {
        ROS_WARN("Failed to compute ESF descriptors for one or both clouds.");
        return false;
    }
    
    double distance = 0.0;
    const pcl::ESFSignature640 &d1 = descriptor1->points[0];
    const pcl::ESFSignature640 &d2 = descriptor2->points[0];
    
    for (int i = 0; i < 640; ++i) {
        double diff = d1.histogram[i] - d2.histogram[i];
        distance += diff * diff;
    }
    distance = std::sqrt(distance);
    
    const double MATCH_THRESHOLD = 0.03;
    
    ROS_INFO("ESF descriptor distance: %f", distance);
    
    return (distance < MATCH_THRESHOLD);
}

std::string
BaseTask::detect_type(const PointCPtr &cloud) {

    if (cloud->empty()) {
        ROS_WARN("PCD empty, defaulting to nought");
        return "nought"; 
    }
    
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    
    for (const auto &point : cloud->points) {
        if (point.x < min_x) { min_x = point.x; }
        if (point.x > max_x) { max_x = point.x; }
        if (point.y < min_y) { min_y = point.y; }
        if (point.y > max_y) { max_y = point.y; }
    }
    
    float length = max_x - min_x;
    float width  = max_y - min_y;
    
    float center_x = (min_x + max_x) / 2.0f;
    float center_y = (min_y + max_y) / 2.0f;
    
    float radius = (length + width) / 9.0f;
    // float radius = diameter / 2.0f;
    
    int count_in_circle = 0;
    for (const auto &point : cloud->points) {
        float dx = point.x - center_x;
        float dy = point.y - center_y;
        float distance = std::sqrt(dx * dx + dy * dy);
        if (distance <= radius) {
            count_in_circle++;
        }
    }

    int total_points = static_cast<int>(cloud->points.size());

    // ROS_INFO("TOTAL");
    // ROS_INFO(std::to_string(total_points).c_str());
    // ROS_INFO("INSIDE");
    // ROS_INFO(std::to_string(count_in_circle).c_str());
    
    return (count_in_circle < (total_points * 0.1)) ? "nought" : "cross";
}

geometry_msgs::PointStamped
BaseTask::get_pcl_centrepoint(const PointCPtr &cloud) {

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);
    geometry_msgs::PointStamped pt;

    pt.header.frame_id = base_frame_;
    pt.header.stamp = ros::Time::now();
    pt.point.x = centroid[0];
    pt.point.y = centroid[1];
    pt.point.z = centroid[2];

    return pt;
}

geometry_msgs::Pose 
BaseTask::pointAndQuatToPose(const geometry_msgs::PointStamped &point, const tf2::Quaternion &quat) {
    geometry_msgs::Pose pose;
    pose.position.x = point.point.x;
    pose.position.y = point.point.y;
    pose.position.z = point.point.z;
    tf2::convert(quat, pose.orientation);
    return pose;
}

void 
BaseTask::removeOutliers(PointCPtr &cloud_ptr, int meanK, double stddevMulThresh) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    PointCPtr filtered(new PointC);
    sor.filter(*filtered);
    cloud_ptr = filtered;
}

bool BaseTask::homingSequence(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& place_pose) {
    // Create retreat pose above the place position
    geometry_msgs::Pose retreat_pose = place_pose;
    
    // Create a vector of waypoints
    std::vector<geometry_msgs::Pose> retreat_waypoints;
    retreat_waypoints.push_back(retreat_pose);

    // Get current pose
    geometry_msgs::Pose current_pose = arm_group_.getCurrentPose().pose;
    
    // Move to start position one axis at a time for safer trajectory
    geometry_msgs::Pose intermediate_pose = retreat_pose;
    if (abs(start_pose.position.x - current_pose.position.x) < abs(start_pose.position.y - current_pose.position.y)) {
        // Y-axis difference is larger, so adjust Y first
        intermediate_pose.position.y = start_pose.position.y;
    } else {
        // X-axis difference is larger, so adjust X first
        intermediate_pose.position.x = start_pose.position.x;
    }
    retreat_waypoints.push_back(intermediate_pose);
    retreat_waypoints.push_back(start_pose);

    // Move the robot through the waypoints
    bool success = moveArmCartesian(retreat_waypoints, 0.01, 0.0);

    if (!success) {
        ROS_ERROR("Failed to complete homing sequence!");
        return false;
    }

    // Close gripper to secure it in home position
    if (!moveGripper(gripper_closed_)) {
        ROS_ERROR("Failed to close gripper during homing sequence!");
        return false;
    }
    
    return true;
}

geometry_msgs::Quaternion
BaseTask::task3Orientation(PointCPtr &cloud_input) {
    if (cloud_input->empty()) {
        ROS_WARN("getMinAreaRectOrientation: Input cloud is empty; returning identity quaternion.");
        geometry_msgs::Quaternion q;
        q.x = 0; q.y = 0; q.z = 0; q.w = 1;
        return q;
    }
    
    pcl::ConvexHull<PointT> ch;
    ch.setInputCloud(cloud_input);
    ch.setDimension(2);
    
    PointC::Ptr hull(new PointC);
    ch.reconstruct(*hull);
    
    if (hull->points.size() < 3) {
        ROS_WARN("Convex hull has fewer than 3 points; returning identity quaternion.");
        geometry_msgs::Quaternion q;
        q.x = 0; q.y = 0; q.z = 0; q.w = 1;
        return q;
    }
    
    double min_area = std::numeric_limits<double>::max();
    double best_angle = 0.0;
    double best_width = 0.0, best_height = 0.0;
    size_t numPoints = hull->points.size();
    
    for (size_t i = 0; i < numPoints; i++) {
        size_t j = (i + 1) % numPoints; 
        
        double dx_edge = hull->points[j].x - hull->points[i].x;
        double dy_edge = hull->points[j].y - hull->points[i].y;
        
        double theta = std::atan2(dy_edge, dx_edge);
        
        double cos_t = std::cos(-theta);
        double sin_t = std::sin(-theta);
        
        double min_x = std::numeric_limits<double>::max();
        double max_x = -std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_y = -std::numeric_limits<double>::max();
        
        for (size_t k = 0; k < numPoints; k++) {
            double x = hull->points[k].x;
            double y = hull->points[k].y;
            double x_rot = x * cos_t - y * sin_t;
            double y_rot = x * sin_t + y * cos_t;
            if (x_rot < min_x) min_x = x_rot;
            if (x_rot > max_x) max_x = x_rot;
            if (y_rot < min_y) min_y = y_rot;
            if (y_rot > max_y) max_y = y_rot;
        }
        
        double width = max_x - min_x;
        double height = max_y - min_y;
        double area = width * height;
        
        if (area < min_area) {
            min_area = area;
            best_angle = theta;
            best_width = width;
            best_height = height;
        }
    }
    
    ROS_INFO("Minimum area rectangle best angle: %f radians, with area = %f", best_angle, min_area);

    square_size = (best_width + best_height) / 2.0;
    
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, best_angle);

    if (obj_type == "cross") {
        tf2::Quaternion additional_rotation;
        additional_rotation.setRPY(0, 0, M_PI / 4); 
        orientation = orientation * additional_rotation;
    }

    geometry_msgs::Quaternion geom_quat;
    tf2::convert(orientation, geom_quat);
    
    return geom_quat;
}

void 
BaseTask::get_grasp_pose(geometry_msgs::PoseStamped &pose_stamped) {

    tf2::Quaternion q;
    tf2::fromMsg(pose_stamped.pose.orientation, q);
    tf2::Vector3 forward_local(1.0, 0.0, 0.0);
    tf2::Vector3 forward_global = tf2::quatRotate(q, forward_local);
    
    forward_global *= square_size/3;
    
    pose_stamped.pose.position.x += forward_global.x();
    pose_stamped.pose.position.y += forward_global.y();
    pose_stamped.pose.position.z += forward_global.z();


    if (obj_type == "nought") {
        tf2::Quaternion currentOrientation;
        tf2::fromMsg(pose_stamped.pose.orientation, currentOrientation);
        
        tf2::Quaternion rot90;
        rot90.setRPY(0, 0, M_PI / 4);
        
        currentOrientation = currentOrientation * rot90;
        pose_stamped.pose.orientation = tf2::toMsg(currentOrientation);
    } else if (obj_type == "cross") {
        tf2::Quaternion currentOrientation;
        tf2::fromMsg(pose_stamped.pose.orientation, currentOrientation);
        
        tf2::Quaternion rot90;
        rot90.setRPY(0, 0, 3 * M_PI / 4);
        
        currentOrientation = currentOrientation * rot90;
        pose_stamped.pose.orientation = tf2::toMsg(currentOrientation);
    }
}

void 
BaseTask::applyOffsetToPose(geometry_msgs::Pose &goal_pose) {
    tf2::Quaternion q_current;
    tf2::fromMsg(goal_pose.orientation, q_current);
    
    tf2::Quaternion q_rotation;
    q_rotation.setRPY(M_PI, 0.0, 0.0);
    
    tf2::Quaternion q_new = q_current * q_rotation;
    q_new.normalize();
    
    goal_pose.orientation = tf2::toMsg(q_new);
}

bool
BaseTask::task3_pick_and_place(geometry_msgs::Pose &orig_pick_pose, geometry_msgs::Pose &orig_goal_pos) {

    geometry_msgs::Pose pick_pose = orig_pick_pose;
    geometry_msgs::Pose goal_pos = orig_goal_pos;
    std::vector<geometry_msgs::Pose> pose_waypoints;

    // Open Gripper
    if (!moveGripper(gripper_open_)) {
        ROS_ERROR("Failed to open gripper");
        return false;
    }

    // Move to pos above shape
    pick_pose.position.z = pick_pose.position.z + 0.2;
    if (!moveArm(pick_pose)) {
        ROS_ERROR("Failed to move to object + z");
        return false;
    }

    ros::Duration(1).sleep();

    // Move down to shape
    pick_pose.position.z = pick_pose.position.z -0.12;
    pose_waypoints.push_back(pick_pose);

    if (!moveArmCartesian(pose_waypoints, 0.01, 0.0)) {
        ROS_ERROR("Failed to move to object");
        return false;
    }

    ros::Duration(2).sleep();

    // Close Gripper on shape
    if (!moveGripper(gripper_closed_)) {
        ROS_ERROR("Failed to close gripper on shape");
        return false;
    }

    ros::Duration(2).sleep();

    // Move up from object
    pick_pose.position.z = pick_pose.position.z + 0.5;
    pose_waypoints.clear();
    pose_waypoints.push_back(pick_pose);

    if (!moveArmCartesian(pose_waypoints, 0.01, 0.0)) {
        ROS_ERROR("Failed to move to object + z");
        return false;
    }

    ros::Duration(2).sleep();
    
    // Move to goal pos + z
    goal_pos.position.z = goal_pos.position.z + 0.5;

    if (!moveArm(goal_pos)) {
        ROS_ERROR("Failed to move to goal");
        return false;
    }

    // Move down to goal
    goal_pos.position.z = goal_pos.position.z - 0.3;
    pose_waypoints.clear();
    pose_waypoints.push_back(goal_pos);

    if (!moveArmCartesian(pose_waypoints, 0.01, 0.0)) {
        ROS_ERROR("Failed to move to Goal");
        return false;
    }

    ros::Duration(2).sleep();

    // Open gripper
    if (!moveGripper(gripper_open_)) {
        ROS_ERROR("Failed to open gripper");
        return false;
    }

    // Re-Home end effector
    goal_pos.position.x = 0.555;
    goal_pos.position.y = 0.0;
    goal_pos.position.z = 0.730;

    if (!moveArm(goal_pos)) {
        ROS_ERROR("Failed to move to Home");
        return false;
    }

}

