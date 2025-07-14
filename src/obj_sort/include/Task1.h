#ifndef TASK1_H_
#define TASK1_H_

#include "BaseTask.h"
#include "cw2_world_spawner/Task1Service.h"

class Task1 : public BaseTask
{
public:
    /// @brief Constructor
    Task1(ros::NodeHandle& nh);
    
    /// @brief Destructor
    ~Task1() = default;
    
    /// @brief Initialize the service
    void initialize();
    
    /// @brief Point cloud callback override
    void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) override;

    /// @brief Callback for Task1 service
    bool t1_callback(cw2_world_spawner::Task1Service::Request &request,
        cw2_world_spawner::Task1Service::Response &response);
    
private:
    /// @brief Service server
    ros::ServiceServer t1_service_;
    
    /// @brief Task1 specific variables
    bool task_1_trigger = false;
};

#endif // TASK1_H_ 