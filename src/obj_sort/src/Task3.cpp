#include "Task3.h"

Task3::Task3(ros::NodeHandle& nh) : BaseTask(nh) {
    ROS_INFO("Task3 initialized");
}

void Task3::initialize() {
    // Register service
    t3_service_ = nh_.advertiseService("/task3_start", &Task3::t3_callback, this);
    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("/task3/total_pub", 1); 
    pcl_pub2 = nh_.advertise<sensor_msgs::PointCloud2>("/task3/total_pub2", 1); 
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/task3/pose_pub", 1); 
    point_pub = nh_.advertise<geometry_msgs::PointStamped>("/task3/goal_pub", 1); 

    ROS_INFO("Task3 service initialized");
}

void Task3::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
    if (task_3_trigger) {
        // Extract input point cloud info
        g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
        pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

        // Get transform to merge point clouds
        try {
            TranBase2World = tfBuffer.lookupTransform("panda_link0", g_input_pc_frame_id_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        // Filtering Point Cloud
        applyVX(g_cloud_ptr, g_cloud_filtered_vx);
        findNormals(g_cloud_filtered_vx);
        segPlane(g_cloud_filtered_vx);
        applyColorFilter(g_cloud_filtered_plane, red_filtered, "red");
        applyColorFilter(g_cloud_filtered_plane, blue_filtered, "blue");
        applyColorFilter(g_cloud_filtered_plane, purp_filtered, "purple");
        applyColorFilter(g_cloud_filtered_plane, black_filtered, "black");
        applyColorFilter(g_cloud_ptr, goal_filtered, "goal");
        
        // Add Segmented PCL to totalcloud_ptr
        addPointsToTotal(red_filtered, totalcloud_ptr);
        addPointsToTotal(blue_filtered, totalcloud_ptr);
        addPointsToTotal(purp_filtered, totalcloud_ptr);

        // Publish global point cloud for visualisationb
        pubPointCloud(pcl_pub, *totalcloud_ptr);

        // Add points to goal cloud
        addPointsToTotal(goal_filtered, total_goal_cloud);
        removeOutliers(total_goal_cloud);
        pubPointCloud(pcl_pub2, *total_goal_cloud);

        // Publish goal cloud centre point for visualisation
        geometry_msgs::PointStamped goal_pt = get_pcl_centrepoint(total_goal_cloud);
        point_pub.publish(goal_pt);

        // Add points to obstacle cloud
        addPointsToTotal(black_filtered, total_obs_cloud);
        // ROS_WARN("Number of points in the point cloud: %lu", static_cast<unsigned long>(total_obs_cloud->points.size()));
        // pubPointCloud(pcl_pub2, *total_obs_cloud);

        // Set flags
        cluster_task_3 = true;
        task_3_trigger = false;
    }
}

bool Task3::t3_callback(cw2_world_spawner::Task3Service::Request &request,
                      cw2_world_spawner::Task3Service::Response &response) {
    ROS_INFO("Task3 service callback triggered");

    std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();
    ROS_INFO("Removing %zu collision objects from the planning scene", object_ids.size());
    planning_scene_interface_.removeCollisionObjects(object_ids);

    std::string floor_collision_name = "floor_coll";
    geometry_msgs::Point floor_centre;
    floor_centre.x = 0.0;
    floor_centre.y = 0.0;
    floor_centre.z = -0.05;
    geometry_msgs::Vector3 floor_size;
    floor_size.x = 2.0;
    floor_size.y = 2.0;
    floor_size.z = 0.1;
    geometry_msgs::Quaternion floor_or;
    floor_or.x = 0.0;
    floor_or.y = 0.0;
    floor_or.z = 0.0;
    floor_or.w = 1.0;

    addCollisionObject(floor_collision_name, floor_centre, floor_size, floor_or);

    // Initialize
    cluster_poses.clear();
    totalcloud_ptr->clear();
    cloud_cluster_arr.clear();
    total_goal_cloud->clear();
    total_obs_cloud->clear();

    // Define the detection poses for different viewpoints
    geometry_msgs::Pose detec_1, detec_2, detec_3, detec_4;
    std::vector<geometry_msgs::Pose> detection;

    // Define the camera rotations
    tf2::Quaternion q_1(-1, 0, 0, 0), q_2, q_3;
    q_2.setRPY(0, 0, M_PI / 4);
    q_3.setRPY(0, 0, -M_PI / 4);
    geometry_msgs::Quaternion detect_Orien_1 = tf2::toMsg(q_1 * q_2);
    geometry_msgs::Quaternion detect_Orien_2 = tf2::toMsg(q_1 * q_3);

    // ------------------------------------------------ Define detection points

    detec_1.position.x = -0.213;
    detec_1.position.y = 0.296;
    detec_1.position.z = 0.8;
    detec_1.orientation = detect_Orien_2;
    detection.push_back(detec_1);

    detec_2.position.x = 0.405;
    detec_2.position.y = 0.189;
    detec_2.position.z = 0.8;
    detec_2.orientation = detect_Orien_1;
    detection.push_back(detec_2);

    detec_3.position.x = 0.405;
    detec_3.position.y = -0.189;
    detec_3.position.z = 0.8;
    detec_3.orientation = detect_Orien_1;
    detection.push_back(detec_3);

    detec_4.position.x = -0.213;
    detec_4.position.y = -0.3;
    detec_4.position.z = 0.8;
    detec_4.orientation = detect_Orien_2;
    detection.push_back(detec_4);


    // ------------------------------------------------ Looping through detection positions

    bool success = false;
    int numb_detection = detection.size(); 
    std::vector<geometry_msgs::Pose> camera_waypoints;

    for (int i = 0; i < numb_detection; i++) {
        // Uncomment for cartesian path planning: faster but chance to not reach goal point
        // camera_waypoints.clear();
        // camera_waypoints.push_back(detection[i]);
        // success = moveArmCartesian(camera_waypoints, 0.01, 0.0);
        // Comment out for cartesian path planning
        success = moveArm(detection.at(i));
        if (success) {
            task_3_trigger = true; // Trigger the task 3 cloud callback
            
            // Add a timeout mechanism and process callbacks
            ros::Time start_time = ros::Time::now();
            ROS_INFO("Waiting for Point Cloud gathering");
            while (!cluster_task_3 && (ros::Time::now() - start_time).toSec() < 10.0) { // 10-second timeout
                ros::spinOnce();  // Process callbacks
                ros::Duration(0.01).sleep();  // Add a small delay to prevent CPU hogging
            }
            if (!cluster_task_3) {
                ROS_ERROR("Timed out waiting for cluster detection!");
                success = false;
            } else {
                success = false;
                cluster_task_3 = false;
            }
        } else {
            ROS_ERROR("Failed to reach the trajectory!");
        }
    }

    // ------------------------------------------------ Cluster obstacles and add their collision

    // Define local vars
    double obs_inflate_boarder = 1.2;

    Cluster(total_obs_cloud, obs_cluster_arr, 0.02);
    int num_obs_clusters = obs_cluster_arr.size();

    PointCPtr obs_cld(new PointC);
    geometry_msgs::Point obs_pt;

    for (int i = 0; i < (num_obs_clusters + 1); i++) {

        if (i == 0) {
            obj_type = "nought";
            obs_cld = total_goal_cloud;
        } else {
            obs_cld = obs_cluster_arr.at(i - 1);
        }

        
        geometry_msgs::PointStamped shape_pt = get_pcl_centrepoint(obs_cld);
        obs_pt = shape_pt.point;
        geometry_msgs::Quaternion obs_or = task3Orientation(obs_cld);

        pcl::PointXYZRGBA min_pt, max_pt;
        pcl::getMinMax3D(*obs_cld, min_pt, max_pt);

        
        geometry_msgs::Vector3 dimensions;
        dimensions.x = (max_pt.x - min_pt.x) * obs_inflate_boarder;
        dimensions.y = (max_pt.y - min_pt.y) * obs_inflate_boarder;
        dimensions.z = (max_pt.z - min_pt.z) * obs_inflate_boarder;

        std::string obs_name = "obs_" + std::to_string(i);

        addCollisionObject(obs_name, obs_pt, dimensions, obs_or);
    }

    // ------------------------------------------------ Cluster point clouds and get additional info

    Cluster(totalcloud_ptr, cloud_cluster_arr, 0.02);

    int num_clusters = cloud_cluster_arr.size();
    int type1_count = 1;
    int type2_count = 0;

    int countNought = 0;
    int countCross = 0;

    // Temporary Loop Vars
    PointCPtr ref_shape_1(new PointC);
    PointCPtr obj_cloud1(new PointC);
    geometry_msgs::PoseStamped poseStamped;

    // List of shape types
    std::vector<std::string> type_list;
    // List of shape poses
    std::vector<geometry_msgs::PoseStamped> pose_list;
    // List of shape sizes
    std::vector<double> size_list;

    for (int i = 0; i < num_clusters; i++) {

        obj_cloud1 = cloud_cluster_arr.at(i);

        // Get shape centrepoint
        geometry_msgs::PointStamped shape_pt = get_pcl_centrepoint(obj_cloud1);

        pcl::PointXYZRGBA min_pt, max_pt;
        pcl::getMinMax3D(*obj_cloud1, min_pt, max_pt);

        geometry_msgs::Vector3 dimensions;
        dimensions.x = (max_pt.x - min_pt.x) * obs_inflate_boarder;
        dimensions.y = (max_pt.y - min_pt.y) * obs_inflate_boarder;
        dimensions.z = (max_pt.z - min_pt.z) * obs_inflate_boarder;

        // Publish pointcloud for visualisation
        // pubPointCloud(pcl_pub, *obj_cloud1);

        // Detect object type
        obj_type = detect_type(obj_cloud1);
        type_list.push_back(obj_type);
        ROS_INFO("Object Type: %s", obj_type.c_str());

        if (obj_type == "nought") {
            ++countNought;
        } else if (obj_type == "cross") {
            ++countCross;
        }

        // Get shape orientation and size
        geometry_msgs::Quaternion shape_or = task3Orientation(obj_cloud1);
        size_list.push_back(square_size);

        // Transform position and orientaiton info to PoseStamped
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = base_frame_; 
        poseStamped.pose.position = shape_pt.point;
        poseStamped.pose.orientation = shape_or;
        get_grasp_pose(poseStamped);
        pose_list.push_back(poseStamped);
        // pose_pub.publish(poseStamped);

        // Normalise, clean up and publish point cloud
        normalizePointCloud(obj_cloud1);
        removeOutliers(obj_cloud1);
        // pubPointCloud(pcl_pub2, *obj_cloud1);

        // Match shape 
        if (i == 0) {
            ref_shape_1 = obj_cloud1;
        } else {
            if (matchShapes(ref_shape_1, obj_cloud1)) {
                ROS_INFO("Shape matches reference");
                type1_count += 1;
            } else {
                ROS_INFO("Shape doesnt match reference");
                type2_count += 1;
            }
        }

        std::string shape_name = "shape_" + std::to_string(i);
        addCollisionObject(shape_name, shape_pt.point, dimensions, shape_or);
    }

    // ------------------------------------------------ Pick and place parameters

    // Get index for largest item of correct shape
    int max_index = -1;
    double max_size = std::numeric_limits<double>::lowest();
    std::string common_shape = (countNought >= countCross) ? "nought" : "cross";

    for (size_t i = 0; i < size_list.size(); ++i) {
        if (type_list[i] == common_shape && size_list[i] > max_size) {
            max_size = size_list[i];
            max_index = i;
        }
    }

    std::string exempt_obj = "shape_" + std::to_string(max_index);
    std::vector<std::string> ex_object_ids;
    ex_object_ids.push_back(exempt_obj);
    planning_scene_interface_.removeCollisionObjects(ex_object_ids);

    poseStamped = pose_list.at(max_index);
    pose_pub.publish(poseStamped);

    // Defining the pick and place poses
    geometry_msgs::PointStamped goal_centre_point = get_pcl_centrepoint(total_goal_cloud);
    geometry_msgs::Pose pick_pose = poseStamped.pose;
    geometry_msgs::Pose place_pose;
    place_pose.position = goal_centre_point.point;

    if (common_shape == "nought") {
        place_pose.orientation.x = 0.0;
        place_pose.orientation.y = 0.0;
        place_pose.orientation.z = 0.382683;
        place_pose.orientation.w = 0.923880;
    } else if (common_shape == "cross") {
        place_pose.orientation.x = 0.0;
        place_pose.orientation.y = 0.0;
        place_pose.orientation.z = 0.923880;
        place_pose.orientation.w = 0.382683;
    } 

    place_pose.position.x = place_pose.position.x + 0.07;

    applyOffsetToPose(pick_pose);
    applyOffsetToPose(place_pose);

    // ------------------------------------------------ Response Management
    response.total_num_shapes = countNought + countCross;
    response.num_most_common_shape = (common_shape == "nought") ? countNought : countCross;

    ROS_WARN("----------RESPONSE----------");
    ROS_WARN("total_num_shapes:");
    ROS_WARN(std::to_string(response.total_num_shapes).c_str());
    ROS_WARN("num_most_common_shape:");
    ROS_WARN(std::to_string(response.num_most_common_shape).c_str());

    // ------------------------------------------------ Move Arm

    bool t3_success = task3_pick_and_place(pick_pose, place_pose);

    return true;
} 