#ifndef TASK3_H_
#define TASK3_H_

#include "BaseTask.h"
#include "Task1.h"
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task3Service.h"

class Task3 : public BaseTask
{
public:
    /// @brief Constructor
    Task3(ros::NodeHandle& nh);
    
    /// @brief Destructor
    ~Task3() = default;
    
    /// @brief Initialize the service
    void initialize();
    
    /// @brief Point cloud callback override
    void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) override;
    
private:
    /// @brief Callback for Task3 service
    bool t3_callback(cw2_world_spawner::Task3Service::Request &request,
                    cw2_world_spawner::Task3Service::Response &response);
    
    /// @brief Service server
    ros::ServiceServer t3_service_;
    
    /// @brief Task3 specific variables
    bool task_3_trigger = false;
    bool cluster_task_3 = false;
    std::vector<geometry_msgs::Pose> cluster_poses;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub2;
    ros::Publisher pose_pub; 
    ros::Publisher point_pub;
};

#endif // TASK3_H_ 