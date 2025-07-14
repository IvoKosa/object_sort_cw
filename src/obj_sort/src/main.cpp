#include <ros/ros.h>
#include <memory>
#include "Task1.h"
#include "Task2.h"
#include "Task3.h"

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "obj_sort_node");
    ros::NodeHandle nh;
    
    // Instantiate the task classes
    std::shared_ptr<Task1> task1 = std::make_shared<Task1>(nh);
    std::shared_ptr<Task2> task2 = std::make_shared<Task2>(nh);
    std::shared_ptr<Task3> task3 = std::make_shared<Task3>(nh);
    
    // Initialize the task services
    task1->initialize();
    task2->initialize();
    task3->initialize();
    
    // Create a ROS subscriber for the input point cloud
    // The cloudCallBack method of each task will only process when their trigger is set
    ros::Subscriber sub_cloud = nh.subscribe("/r200/camera/depth_registered/points",
                                             1,
                                             &Task1::cloudCallBack,
                                             task1.get());

    ros::Subscriber sub_cloud2 = nh.subscribe("/r200/camera/depth_registered/points",
                                             1,
                                             &Task2::cloudCallBack,
                                             task2.get());

    ros::Subscriber sub_cloud3 = nh.subscribe("/r200/camera/depth_registered/points",
                                             1,
                                             &Task3::cloudCallBack,
                                             task3.get());
    
    // MoveIt! requirement for non-blocking group.move()
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Loop rate in Hz
    ros::Rate rate(10);
    
    ROS_INFO("CW2 solution node initialized and ready");
    
    while (ros::ok()) {
        // Spin and process all pending callbacks
        ros::spinOnce();
        
        // Sleep to fulfill the loop rate
        rate.sleep();
    }
    
    return 0;
} 