#ifndef TASK2_H_
#define TASK2_H_

#include "BaseTask.h"
#include "cw2_world_spawner/Task2Service.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * @class Task2
 * @brief Handles identification of noughts and crosses and matches mystery objects to reference objects
 * 
 * Task2 is responsible for:
 * 1. Identifying shapes of objects (noughts and crosses) using point cloud processing
 * 2. Planning and executing optimal paths to inspect mystery and reference objects
 * 3. Determining which reference object matches the mystery object
 * 
 * The class uses distance-based path planning to minimize robot arm movement.
 */
class Task2 : public BaseTask
{
public:
    /**
     * @brief Constructor
     * @param nh The ROS node handle for communication
     */
    Task2(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~Task2() = default;
    
    /**
     * @brief Initialize the service server
     * 
     * Advertises the task2_start service that clients can call to initiate the task.
     */
    void initialize();
    
    /**
     * @brief Processes point cloud data to identify object shapes
     * @param cloud_input_msg The input point cloud message
     * 
     * Overrides the base class method to implement specialized processing for
     * nought and cross identification. Only processes point clouds when task_2_trigger
     * is active.
     */
    void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) override;
    
private:
    /**
     * @brief Visualize objects with markers in RViz
     * @param positions Vector of object positions to visualize
     * @param types Vector of object types ("nought" or "cross")
     * 
     * Creates RViz markers to visualize the objects at their positions.
     * This helps with debugging and verification of object recognition.
     */
    void visualizeObjects(const std::vector<geometry_msgs::Point>& positions, 
                         const std::vector<std::string>& types);

    /**
     * @brief Moves the arm to inspect an object and identify its shape
     * @param target_pose The pose to move the arm to
     * @param object_type Reference to store the recognized object type
     * @param description Description of the object for error messages
     * @return True if successful, false if movement or recognition failed
     * 
     * This helper function combines arm movement and object recognition into
     * a single operation with error handling and timeout.
     */
    bool moveAndRecognize(const geometry_msgs::Pose& target_pose, 
                          std::string& object_type, 
                          const std::string& description);
    
    /**
     * @brief Calculates the Euclidean distance between two 3D points
     * @param p1 First point
     * @param p2 Second point
     * @return The distance between the points
     * 
     * Used for distance-based path planning to minimize robot movement.
     */
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    
    /**
     * @brief Service callback for the task2_start service
     * @param request Contains positions of mystery and reference objects
     * @param response The service response to fill with results
     * @return True if task completed successfully, false otherwise
     * 
     * This is the main entry point for Task2 execution. It:
     * 1. Plans the optimal inspection path based on object distances
     * 2. Moves the arm to identify objects
     * 3. Determines which reference object matches the mystery object
     */
    bool t2_callback(cw2_world_spawner::Task2Service::Request &request,
                     cw2_world_spawner::Task2Service::Response &response);
    
    // Member variables
    ros::ServiceServer t2_service_;     ///< Service server for task2_start
    ros::Publisher g_pub_color_filtered; ///< Publisher for color-filtered point cloud
    ros::Publisher marker_pub_;         ///< Publisher for visualization markers
    bool task_2_trigger = false;        ///< Flag to trigger point cloud processing
    bool recog_task_2 = false;          ///< Flag to indicate object recognition is complete
    std::string current_object_type;    ///< Stores the currently identified object type (nought/cross)
    std::string current_object_color;   ///< Stores the currently identified object color
    int object_size_ = 30;              ///< Stores the estimated object size in mm (20, 30, or 40)
    bool mystery_obj_flag;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub2;
    geometry_msgs::Pose initial_pose_; // Stores the initial pose of the arm
};

#endif // TASK2_H_ 