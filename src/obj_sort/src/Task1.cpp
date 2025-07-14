#include "Task1.h"

Task1::Task1(ros::NodeHandle& nh) : BaseTask(nh) {
    ROS_INFO("Task1 initialized");
}

void Task1::initialize() {
    // Register service
    t1_service_ = nh_.advertiseService("/task1_start", &Task1::t1_callback, this);
    ROS_INFO("Task1 service initialized");
}

void Task1::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
    if (task_1_trigger) {
        // Extract input point cloud info
        g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
        pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

        // Downsampling the point cloud
        applyVX(g_cloud_ptr, g_cloud_filtered_vx);

        // Segment the plane
        findNormals(g_cloud_filtered_vx);
        segPlane(g_cloud_filtered_vx);

        // Project the filtered point cloud on the plane
        projection(g_cloud_filtered_plane);
        pubFilteredPCMsg(g_pub_rm_plane, *g_cloud_projected_plane);

        // Calculate the orientation of the object
        q_object_task_1 = getOrientation(g_cloud_projected_plane);

        // Set flags
        segment_done = true;
        task_1_trigger = false;
    }
}

bool Task1::t1_callback(cw2_world_spawner::Task1Service::Request &request,
                      cw2_world_spawner::Task1Service::Response &response) {
    ROS_INFO("Task1 service callback triggered");

    //start position
    geometry_msgs::Pose start_pose = arm_group_.getCurrentPose().pose;
    
    // Remove all the collisions in the path planning
    remove_all_collisions();

    // Add the plane for the path planning
    add_plane();

    // Get the object type
    obj_type = request.shape_type;

    // Get the object and goal points from request
    geometry_msgs::Point object_point = request.object_point.point;
    geometry_msgs::Point goal_point = request.goal_point.point;

    // Define the detection pose
    geometry_msgs::Pose detect_pose;
    detect_pose.position = link2Cam(object_point);
    detect_pose.position.z = object_point.z + 0.6;

    // Set orientation for detection
    tf2::Quaternion q_1(-1, 0, 0, 0), q_2;
    q_2.setRPY(0, 0, M_PI / 4);
    geometry_msgs::Quaternion detect_orien = tf2::toMsg(q_1 * q_2);
    detect_pose.orientation = detect_orien;

    // Move the robot to the detect pose
    bool success = false;

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose = arm_group_.getCurrentPose().pose;
    geometry_msgs::Pose detect_pose_single_axis = detect_pose;

    if (abs(detect_pose.position.x - current_pose.position.x) < abs(detect_pose.position.y - current_pose.position.y)) {
        detect_pose_single_axis.position.y = current_pose.position.y;
    } else {
        detect_pose_single_axis.position.x = current_pose.position.x;
    }
    
    waypoints.push_back(detect_pose_single_axis);
    waypoints.push_back(detect_pose);

    success = moveArmCartesian(waypoints, 0.01, 0.0);

    if (success) {
        // Enable the segment plane trigger when the robot is at the detection position
        task_1_trigger = true;
    } else {
        ROS_ERROR("Failed to reach the trajectory!");
        return false;
    }

    // Add a timeout mechanism and process callbacks
    ros::Time start_time = ros::Time::now();
    ROS_INFO("Waiting for point cloud processing...");
    while (!segment_done && (ros::Time::now() - start_time).toSec() < 10.0) { // 10-second timeout
        ros::spinOnce();  // Process callbacks
        ros::Duration(0.01).sleep();  // Add a small delay to prevent CPU hogging
    }

    // if (!segment_done) {
    //     ROS_ERROR("Timed out waiting for point cloud processing!");
    //     return false;
    // }

    segment_done = false;
    detect_orien = tf2::toMsg(q_object_task_1);
    detect_pose.orientation = detect_orien;
    detect_pose.position = cam2Link(detect_pose.position);

    
    // Shape size configuration
    // The x value from the image determines the cell size of the grid:
    // - 20mm (0.02m) -> 100mm square shape
    // - 30mm (0.03m) -> 150mm square shape
    // - 40mm (0.04m) -> 200mm square shape
    constexpr double DEFAULT_CELL_SIZE = 0.04;  // 40mm in meters
    
    // Define shape parameters based on geometric properties of the 5x5 grid
    struct ShapeParams {
        double graspOffsetRatio;  // Offset as a multiplier of cell size
        const char* name;
        double minSize;  // Minimum supported size in meters
        double maxSize;  // Maximum supported size in meters
    };
    
    // Define parameters for each shape type
    // The grasp offset ratio is determined by the geometry of the shape. 
    // "nought": Optimal grasp point is at edge of hollow center (2 cells from center)
    // "cross": Optimal grasp point is on one of the arms (1.5 cells from center)
    const std::map<std::string, ShapeParams> shapeParams = {
        {"nought", {2.0, "Nought (O)", 0.02, 0.04}},  // Supports 20mm to 40mm cells
        {"cross", {1.5, "Cross (X)", 0.02, 0.04}}     // Supports 20mm to 40mm cells
    };
    
    // For Task1, we use the default size. For other tasks, this could be determined
    // from the object properties or passed as a parameter
    double cell_size = DEFAULT_CELL_SIZE;
    
    // Extract size from object name if available (format: "shape_color_XXmm")
    // This allows future tasks to use different sizes
    size_t mm_pos = obj_type.find("mm");
    if (mm_pos != std::string::npos) {
        size_t size_pos = obj_type.rfind('_');
        if (size_pos != std::string::npos && size_pos < mm_pos) {
            std::string size_str = obj_type.substr(size_pos + 1, mm_pos - size_pos - 1);
            try {
                double size_mm = std::stod(size_str);
                cell_size = size_mm / 1000.0;  // Convert mm to meters
                ROS_INFO("Detected object size: %.1f mm (%.3f m)", size_mm, cell_size);
            } catch (const std::exception& e) {
                ROS_WARN("Failed to parse object size, using default: %s", e.what());
            }
        }
    }
    
    // Calculate grasp offset based on shape type and size
    double rad_offset = 0.0;
    
    // Look up shape parameters and validate size
    auto it = shapeParams.find(obj_type.substr(0, obj_type.find('_')));
    if (it != shapeParams.end()) {
        const ShapeParams& params = it->second;
        if (cell_size < params.minSize || cell_size > params.maxSize) {
            ROS_WARN("Object size %.3fm outside supported range [%.3f, %.3f], using clamped value",
                    cell_size, params.minSize, params.maxSize);
            // Manual clamping implementation
            cell_size = std::min(std::max(cell_size, params.minSize), params.maxSize);
        }
        rad_offset = cell_size * params.graspOffsetRatio;
        ROS_INFO("Using %s shape (size: %.3fm) with grasp offset: %.3fm", 
                params.name, cell_size, rad_offset);
    } else {
        ROS_WARN("Unknown shape type: %s, using default grasp offset", obj_type.c_str());
    }
    
    detect_pose.position.x = detect_pose.position.x + sin(rad) * rad_offset;
    detect_pose.position.y = detect_pose.position.y + cos(rad) * rad_offset;

    // Create pick pose with the new orientation
    geometry_msgs::Pose pick_pose = detect_pose;
    pick_pose.position.z = 0.145;  // Grasp height
    
    // Create place pose
    geometry_msgs::Pose place_pose;
    place_pose.orientation = tf2::toMsg(q_1 * q_2);
    place_pose.position = goal_point;
    place_pose.position.y += 0.05;
    place_pose.position.z = 0.145;  // Place height

    geometry_msgs::Pose retreat_pose = place_pose;
    retreat_pose.position.z += 0.3;
    
    // Use the enhanced pick and place routine with corner navigation
    double approach_height = 0.25;  // Height above objects for approach
    success = pickAndPlaceRoutine(pick_pose, place_pose, approach_height);
    
    if (!success) {
        ROS_ERROR("Pick and place routine failed!");
        return false;
    }

    // Use the new homing sequence instead of the inline retreat code
    success = homingSequence(start_pose, retreat_pose);
    if (!success) {
        return false;
    }
    
    // Remove all the collisions in the path planning
    remove_all_collisions();

    return true;
} 
