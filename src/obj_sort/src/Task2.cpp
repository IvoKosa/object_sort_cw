#include "Task2.h"

/**
 * @brief Constructor for Task2
 * @param nh Node handle for ROS communication
 */
Task2::Task2(ros::NodeHandle& nh) : BaseTask(nh) {
}

/**
 * @brief Initialize the task service
 * 
 * Sets up the service server for task2_start to handle service requests.
 */
void Task2::initialize() {
    // Initialize publishers for filtered point cloud visualization
    g_pub_rm_plane = nh_.advertise<sensor_msgs::PointCloud2>("/task2/filtered_cloud", 1);
    g_pub_color_filtered = nh_.advertise<sensor_msgs::PointCloud2>("/task2/color_filtered_cloud", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/task2/object_markers", 1);

    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("/task3/total_pub", 1); 
    pcl_pub2 = nh_.advertise<sensor_msgs::PointCloud2>("/task3/total_pub2", 1); 
    
    // Register service
    t2_service_ = nh_.advertiseService("/task2_start", &Task2::t2_callback, this);
}

/**
 * @brief Process point cloud data to identify object shapes
 * @param cloud_input_msg Input point cloud message
 * 
 * This callback processes the incoming point cloud data when triggered.
 * It extracts, filters, and analyzes the point cloud to determine if the
 * observed object is a nought or cross.
 */
void Task2::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
    // Early exit if not triggered
    if (!task_2_trigger) return;

    // Extract and convert point cloud data
    g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
    pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

    // Apply voxel grid filter, find normals, and segment the plane
    applyVX(g_cloud_ptr, g_cloud_filtered_vx);
    
    // Apply color filtering - filter the point cloud by color
    // We need to check all three possible colors (blue, red, purple)
    // and choose the one that returns the most points
    PointCPtr cloud_filtered_blue(new PointC);
    PointCPtr cloud_filtered_red(new PointC);
    PointCPtr cloud_filtered_purple(new PointC);
    
    // Apply different color filters
    applyColorFilter(g_cloud_filtered_vx, cloud_filtered_blue, "blue");
    applyColorFilter(g_cloud_filtered_vx, cloud_filtered_red, "red");
    applyColorFilter(g_cloud_filtered_vx, cloud_filtered_purple, "purple");
    
    // Choose the color with the most points
    size_t blue_points = cloud_filtered_blue->size();
    size_t red_points = cloud_filtered_red->size();
    size_t purple_points = cloud_filtered_purple->size();
    
    // Identify the dominant color
    std::string detected_color;
    PointCPtr best_color_cloud(new PointC);
    
    if (blue_points >= red_points && blue_points >= purple_points) {
        *g_cloud_filtered_vx = *cloud_filtered_blue;
        *best_color_cloud = *cloud_filtered_blue;
        detected_color = "blue";
    } else if (red_points >= blue_points && red_points >= purple_points) {
        *g_cloud_filtered_vx = *cloud_filtered_red;
        *best_color_cloud = *cloud_filtered_red;
        detected_color = "red";
    } else {
        *g_cloud_filtered_vx = *cloud_filtered_purple;
        *best_color_cloud = *cloud_filtered_purple;
        detected_color = "purple";
    }
    
    // Estimate object size from bounding box dimensions
    if (best_color_cloud->size() > 0) {
        PointT min_point, max_point;
        pcl::getMinMax3D(*best_color_cloud, min_point, max_point);
        
        // Calculate approximate width and height in meters
        double width = max_point.x - min_point.x;
        double height = max_point.y - min_point.y;
        
        // Use the larger dimension to estimate size
        double max_dimension = std::max(width, height);
        
        // Classify into one of the three size categories
        std::string size_category;
        if (max_dimension < 0.08) {
            size_category = "20mm";
            object_size_ = 20;
        } else if (max_dimension < 0.12) {
            size_category = "30mm";
            object_size_ = 30;
        } else {
            size_category = "40mm";
            object_size_ = 40;
        }
    }
    
    // Check if we have valid point clouds to work with
    if (best_color_cloud->empty()) {
        ROS_WARN("No color-filtered points found. Cannot determine object type.");
        current_object_type = "cross"; // Default to cross if no points
        current_object_color = detected_color;
        
        if (mystery_obj_flag) {
            *task2_mystery_obj = *best_color_cloud;
            mystery_obj_flag = false;
            pubPointCloud(pcl_pub, *best_color_cloud);
        } else {
            *task2_known_obj = *best_color_cloud;
            mystery_obj_flag = true;
            pubPointCloud(pcl_pub2, *best_color_cloud);
        }
        
        // Update state flags and exit
        recog_task_2 = true;
        task_2_trigger = false;
        return;
    }
    
    // Publish the color-filtered point cloud to RViz
    pubFilteredPCMsg(g_pub_color_filtered, *best_color_cloud);
    
    // Continue with normal processing
    findNormals(g_cloud_filtered_vx);
    segPlane(g_cloud_filtered_vx);
    
    // Check if we have valid plane-filtered points
    if (g_cloud_filtered_plane->empty()) {
        ROS_WARN("No valid points after plane segmentation. Using color-filtered cloud for identification.");
        
        // Use the color-filtered cloud for identification instead
        *g_cloud_projected_plane = *best_color_cloud;
        
        // Try to determine object type directly from the color-filtered cloud
        // current_object_type = getObjectType(best_color_cloud);
        // current_object_color = detected_color;
        
        if (mystery_obj_flag) {
            normalizePointCloud(best_color_cloud);
            *task2_mystery_obj = *best_color_cloud;
            mystery_obj_flag = false;
            pubPointCloud(pcl_pub, *best_color_cloud);
        } else {
            normalizePointCloud(best_color_cloud);
            *task2_known_obj = *best_color_cloud;
            mystery_obj_flag = true;
            pubPointCloud(pcl_pub2, *best_color_cloud);
        }
        
        // Update state flags and exit
        recog_task_2 = true;
        task_2_trigger = false;
        return;
    }
    
    // Project points onto the plane
    try {
        projection(g_cloud_filtered_plane);
        
        // Check if projection worked
        if (g_cloud_projected_plane->empty()) {
            throw std::runtime_error("Projection resulted in empty point cloud");
        }
        
        // Apply pass-through filters to isolate object
        double thres = 0.175;
        applyPT_x(g_cloud_projected_plane, g_cloud_projected_plane, thres);
        applyPT_y(g_cloud_projected_plane, g_cloud_projected_plane, thres);
        
        // Determine object type
        // current_object_type = getObjectType(g_cloud_projected_plane);
        // current_object_color = detected_color;
        
    } catch (const std::exception& e) {
        ROS_WARN("Error during projection or filtering: %s", e.what());
        ROS_WARN("Using color-filtered cloud for identification instead.");
        
        // Use the color-filtered cloud for identification
        // current_object_type = getObjectType(best_color_cloud);
        // current_object_color = detected_color;
    }

    if (mystery_obj_flag) {
        normalizePointCloud(best_color_cloud);
        *task2_mystery_obj = *best_color_cloud;
        mystery_obj_flag = false;

        // Publish PCL
        pubPointCloud(pcl_pub, *best_color_cloud);
    } else {
        normalizePointCloud(best_color_cloud);
        *task2_known_obj = *best_color_cloud;
        mystery_obj_flag = true;

        // Publish PCL
        pubPointCloud(pcl_pub2, *best_color_cloud);
    }
    
    // Update state flags
    recog_task_2 = true;
    task_2_trigger = false;
}

/**
 * @brief Move the robot arm to a pose and recognize the object there
 * @param target_pose Target pose for the arm
 * @param object_type Reference to store the recognized object type
 * @param description Description of the object for error messages
 * @return True if successful, false if movement or recognition failed
 * 
 * This function moves the arm to the specified position, then triggers
 * point cloud processing to identify the object at that location.
 */
bool Task2::moveAndRecognize(const geometry_msgs::Pose& target_pose, std::string& object_type, const std::string& description) {

    ROS_WARN("MOVING TO: ");
    ROS_WARN(description.c_str());
    //waypoints
    std::vector<geometry_msgs::Pose> waypoints;

    // Get current pose
    geometry_msgs::Pose current_pose = arm_group_.getCurrentPose().pose;
    current_pose.position.z = 0.5;
    geometry_msgs::Pose lower_target_pose = target_pose;
    lower_target_pose.position.z -= 0.2;
    
    // Move to start position one axis at a time for safer trajectory
    geometry_msgs::Pose intermediate_pose = target_pose;
    intermediate_pose.position.z -= 0.2;
    if (abs(current_pose.position.x - target_pose.position.x) < abs(current_pose.position.y - target_pose.position.y)) {
        // Y-axis difference is larger, so adjust Y first
        intermediate_pose.position.y = current_pose.position.y;
    } else {
        // X-axis difference is larger, so adjust X first
        intermediate_pose.position.x = current_pose.position.x;
    }
    waypoints.push_back(intermediate_pose);
    waypoints.push_back(lower_target_pose);
    waypoints.push_back(target_pose);
    // Move arm to target position
    if (!moveArmCartesian(waypoints, 0.01, 0.0)) {
        // Keeping this error message as it's essential for debugging
        ROS_ERROR("Failed to reach trajectory");
        return false;
    }
    
    // Trigger point cloud processing
    task_2_trigger = true;
    
    // Wait for recognition to complete with timeout
    const double timeout = 10.0;
    ros::Time start_time = ros::Time::now();
    
    while (!recog_task_2 && (ros::Time::now() - start_time).toSec() < timeout) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    
    // Check for timeout
    if (!recog_task_2) {
        // Keeping this error message as it's essential for debugging
        ROS_ERROR("Recognition timeout");
        return false;
    }
    
    // Store the object type and reset flag
    object_type = current_object_type;
    recog_task_2 = false;
    
    return true;
}

/**
 * @brief Calculate Euclidean distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Euclidean distance
 */
double Task2::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + 
                    std::pow(p1.y - p2.y, 2) + 
                    std::pow(p1.z - p2.z, 2));
}

/**
 * @brief Callback for the Task2 service
 * @param request Service request containing object positions
 * @param response Service response to be filled
 * @return True if successful, false otherwise
 * 
 * This is the main function for Task2 which:
 * 1. Identifies the most efficient path to visit the mystery object and a reference
 * 2. Moves the arm to each object and identifies their shapes
 * 3. Determines which reference object matches the mystery object
 */
bool Task2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
                        cw2_world_spawner::Task2Service::Response &response) {

    // Print task start message using WARN level to ensure visibility
    ROS_WARN("Starting Task 2");

    // Get current pose
    geometry_msgs::Pose start_pose = arm_group_.getCurrentPose().pose;

    task2_mystery_obj->clear();
    task2_known_obj->clear();

    // Setup object poses
    geometry_msgs::Pose ref_1, ref_2, mystery, detectPose, targetPose;
    ref_1.position = request.ref_object_points[0].point;
    ref_2.position = request.ref_object_points[1].point;
    mystery.position = request.mystery_object_point.point;
    std::string ref_1_type, ref_2_type, mystery_type;
    
    // Create vectors for object visualization
    std::vector<geometry_msgs::Point> positions = {
        ref_1.position, ref_2.position, mystery.position
    };
    std::vector<std::string> types; // We'll fill this during inspection
    
    // Prepare common orientation for detection poses
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI / 4);
    targetPose.orientation = detectPose.orientation = tf2::toMsg(q);

    // Get current arm position and calculate distances to all objects
    geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
    
    // Calculate distances from arm to each object
    double dist_to_ref1 = calculateDistance(current_pose.pose.position, ref_1.position);
    double dist_to_ref2 = calculateDistance(current_pose.pose.position, ref_2.position);
    double dist_to_mystery = calculateDistance(current_pose.pose.position, mystery.position);
    
    // Calculate distances between objects for optimal path planning
    double dist_ref1_to_mystery = calculateDistance(ref_1.position, mystery.position);
    double dist_ref2_to_mystery = calculateDistance(ref_2.position, mystery.position);
    
    // Determine optimal inspection order (which object to visit first)
    bool inspect_mystery_first = (dist_to_mystery <= dist_to_ref1 && dist_to_mystery <= dist_to_ref2);
    
    // Determine which reference to visit for most efficient path
    int closest_ref_idx;
    
    if (inspect_mystery_first) {
        // If visiting mystery first, choose reference closest to the mystery object
        closest_ref_idx = (dist_ref1_to_mystery <= dist_ref2_to_mystery) ? 0 : 1;
    } else {
        // If visiting a reference first, choose reference closest to current arm position
        closest_ref_idx = (dist_to_ref1 <= dist_to_ref2) ? 0 : 1;
    }
    
    // Transform object positions to camera frame and prepare detection poses
    geometry_msgs::Point ref_point_cam = link2Cam(request.ref_object_points[closest_ref_idx].point);
    geometry_msgs::Point mystery_point_cam = link2Cam(mystery.position);
    
    detectPose.position = ref_point_cam;
    detectPose.position.z = 0.66;  // Fixed camera height for detection
    targetPose.position = mystery_point_cam;
    targetPose.position.z = 0.66;  // Fixed camera height for detection
    
    // Visualize objects in RViz initially with unknown types (will update later)
    types = {"unknown", "unknown", "unknown"};
    visualizeObjects(positions, types);
    
    // Execute inspection in optimal order
    std::string inspected_ref_type;
    int inspected_ref_idx;
    
    // Check if the point clouds were properly captured
    bool capture_failed = false;
    
    try {
        if (inspect_mystery_first) {
            mystery_obj_flag = true;

            // First inspect mystery object
            if (!moveAndRecognize(targetPose, mystery_type, "mystery object")) {
                capture_failed = true;
            } else {
                // Update visualization with recognized mystery type
                types[2] = mystery_type;
                visualizeObjects(positions, types);
                
                // Then inspect chosen reference object
                if (!moveAndRecognize(detectPose, inspected_ref_type, 
                                    "reference " + std::to_string(closest_ref_idx + 1))) {
                    capture_failed = true;
                } else {
                    // Store which reference we inspected
                    inspected_ref_idx = closest_ref_idx;
                    if (closest_ref_idx == 0) {
                        ref_1_type = inspected_ref_type;
                    } else {
                        ref_2_type = inspected_ref_type;
                    }
                    
                    // Update visualization with recognized reference type
                    types[closest_ref_idx] = inspected_ref_type;
                    visualizeObjects(positions, types);
                }
            }
        } else {
            mystery_obj_flag = false;

            // First inspect chosen reference object
            if (!moveAndRecognize(detectPose, inspected_ref_type, 
                                "reference " + std::to_string(closest_ref_idx + 1))) {
                capture_failed = true;
            } else {
                // Store which reference we inspected
                inspected_ref_idx = closest_ref_idx;
                if (closest_ref_idx == 0) {
                    ref_1_type = inspected_ref_type;
                } else {
                    ref_2_type = inspected_ref_type;
                }
                
                // Update visualization with recognized reference type
                types[closest_ref_idx] = inspected_ref_type;
                visualizeObjects(positions, types);
                
                // Then inspect mystery object
                if (!moveAndRecognize(targetPose, mystery_type, "mystery object")) {
                    capture_failed = true;
                } else {
                    // Update visualization with recognized mystery type
                    types[2] = mystery_type;
                    visualizeObjects(positions, types);
                }
            }
        }

        // If we failed to capture any of the objects, we need to return a default response
        if (capture_failed) {
            ROS_WARN("Capture failed for at least one object. Making a best guess...");
            response.mystery_object_num = 1; // Default to first reference
            return true;
        }

        // Infer the shape of the unvisited reference (it must be different from the inspected one)
        std::string uninspected_ref_type = (inspected_ref_type == "cross") ? "nought" : "cross";
        int uninspected_ref_idx = (inspected_ref_idx == 0) ? 1 : 0;
        
        // Store the inferred type for the uninspected reference
        if (uninspected_ref_idx == 0) {
            ref_1_type = uninspected_ref_type;
        } else {
            ref_2_type = uninspected_ref_type;
        }
        
        // Update visualization for inferred reference
        types[uninspected_ref_idx] = uninspected_ref_type;
        visualizeObjects(positions, types);

        // Determine if the shapes match using PCL data
        bool shapes_match = false;
        
        // Check if our point clouds have sufficient points for comparison
        if (task2_mystery_obj->size() < 10 || task2_known_obj->size() < 10) {
            ROS_WARN("Insufficient point cloud data for matching. Using type-based matching.");
            shapes_match = (mystery_type == inspected_ref_type);
        } else {
            // Use detailed shape matching with sufficient points
            shapes_match = matchShapes(task2_mystery_obj, task2_known_obj);
        }
        
        // Set the response based on which reference matches the mystery object
        if (shapes_match) {
            response.mystery_object_num = inspected_ref_idx + 1;
        } else {
            response.mystery_object_num = uninspected_ref_idx + 1;
        }
        
        // Only print the final result message using WARN level to ensure visibility
        ROS_WARN("Mystery object matches reference %ld", response.mystery_object_num);
        
    } catch (const std::exception& e) {
        // Handle any exceptions that might occur
        ROS_ERROR("Exception during task execution: %s", e.what());
        // Provide a default answer
        response.mystery_object_num = 1;
    }   

    // Get current pose
    current_pose = arm_group_.getCurrentPose();
    geometry_msgs::Pose current_pose_unstamped = current_pose.pose;
    current_pose_unstamped.position.z = 0.5;

    // homing sequence
    if (!homingSequence(start_pose, current_pose_unstamped)) {
        ROS_ERROR("Failed to homing sequence!");
        return false;
    }

    // Only print the final result message using WARN level to ensure visibility
    ROS_WARN("Mystery object matches reference %ld", response.mystery_object_num);
    
    return true;
}

/**
 * @brief Visualize objects with markers in RViz
 * @param positions Vector of object positions to visualize
 * @param types Vector of object types ("nought" or "cross")
 * 
 * Creates RViz markers to visualize the objects at their positions.
 * This helps with debugging and verification of object recognition.
 */
void Task2::visualizeObjects(const std::vector<geometry_msgs::Point>& positions, 
                            const std::vector<std::string>& types) {
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < positions.size(); i++) {
        // Determine marker size based on object position z-coordinate
        // This is a rough estimation of the object size category (20mm, 30mm, or 40mm)
        double size_factor = 0.03; // Default to 30mm (middle size)
        
        if (i < types.size()) {
            std::string object_type = types[i];
            
            // Base position for the marker
            geometry_msgs::Point base_pos = positions[i];
            
            // Color based on object position (which can represent the different sizes)
            std_msgs::ColorRGBA color;
            color.a = 0.8; // Alpha (transparency)
            
            // Try to estimate the color from position
            // In a real scenario, you'd use the detected color
            if (i == 0) {
                // Reference 1 - Blue
                color.r = 0.0; color.g = 0.0; color.b = 1.0;
            } else if (i == 1) {
                // Reference 2 - Red
                color.r = 1.0; color.g = 0.0; color.b = 0.0;
            } else {
                // Mystery object - Purple
                color.r = 0.8; color.g = 0.0; color.b = 0.8;
            }
            
            // Create markers based on object type
            if (object_type == "nought") {
                // For nought, create a hollow square (5x5 grid with empty center)
                const int grid_size = 5;
                double cell_size = size_factor / grid_size;
                
                // Create the perimeter cells (outer border)
                for (int row = 0; row < grid_size; row++) {
                    for (int col = 0; col < grid_size; col++) {
                        // Only create markers for the perimeter cells
                        if (row == 0 || row == grid_size-1 || col == 0 || col == grid_size-1) {
                            visualization_msgs::Marker cell;
                            cell.header.frame_id = "panda_link0";
                            cell.header.stamp = ros::Time::now();
                            cell.ns = "nought_cells";
                            cell.id = i * 100 + row * grid_size + col;
                            cell.type = visualization_msgs::Marker::CUBE;
                            
                            // Position the cell in the grid
                            cell.pose.position = base_pos;
                            cell.pose.position.x += (col - grid_size/2.0) * cell_size;
                            cell.pose.position.y += (row - grid_size/2.0) * cell_size;
                            
                            cell.pose.orientation.w = 1.0;
                            cell.scale.x = cell_size * 0.9; // Slightly smaller than cell
                            cell.scale.y = cell_size * 0.9;
                            cell.scale.z = 0.005; // Thin height
                            
                            cell.color = color;
                            cell.lifetime = ros::Duration(0);
                            
                            marker_array.markers.push_back(cell);
                        }
                    }
                }
            } else if (object_type == "cross") {
                // For cross, create a plus sign (5x5 grid with filled middle row and column)
                const int grid_size = 5;
                double cell_size = size_factor / grid_size;
                
                // Create the cross pattern cells
                for (int row = 0; row < grid_size; row++) {
                    for (int col = 0; col < grid_size; col++) {
                        // Only create markers for the middle row and column
                        if (row == grid_size/2 || col == grid_size/2) {
                            visualization_msgs::Marker cell;
                            cell.header.frame_id = "panda_link0";
                            cell.header.stamp = ros::Time::now();
                            cell.ns = "cross_cells";
                            cell.id = i * 100 + row * grid_size + col;
                            cell.type = visualization_msgs::Marker::CUBE;
                            
                            // Position the cell in the grid
                            cell.pose.position = base_pos;
                            cell.pose.position.x += (col - grid_size/2.0) * cell_size;
                            cell.pose.position.y += (row - grid_size/2.0) * cell_size;
                            
                            cell.pose.orientation.w = 1.0;
                            cell.scale.x = cell_size * 0.9; // Slightly smaller than cell
                            cell.scale.y = cell_size * 0.9;
                            cell.scale.z = 0.005; // Thin height
                            
                            cell.color = color;
                            cell.lifetime = ros::Duration(0);
                            
                            marker_array.markers.push_back(cell);
                        }
                    }
                }
            } else {
                // For unknown type, just show a sphere
                visualization_msgs::Marker marker;
                marker.header.frame_id = "panda_link0";
                marker.header.stamp = ros::Time::now();
                marker.ns = "unknown_objects";
                marker.id = i;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.pose.position = base_pos;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = size_factor;
                marker.scale.y = size_factor;
                marker.scale.z = size_factor;
                marker.color = color;
                marker.lifetime = ros::Duration(0);
                
                marker_array.markers.push_back(marker);
                
                // Add a text marker to indicate it's unknown
                visualization_msgs::Marker text;
                text.header.frame_id = "panda_link0";
                text.header.stamp = ros::Time::now();
                text.ns = "object_labels";
                text.id = i + positions.size();
                text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text.pose.position = base_pos;
                text.pose.position.z += 0.03; // Position text above object
                text.text = "Unknown";
                text.scale.z = 0.01; // Text height
                text.color.r = 1.0;
                text.color.g = 1.0;
                text.color.b = 1.0;
                text.color.a = 1.0;
                text.lifetime = ros::Duration(0);
                
                marker_array.markers.push_back(text);
            }
        }
    }
    
    // Publish marker array
    marker_pub_.publish(marker_array);
}
