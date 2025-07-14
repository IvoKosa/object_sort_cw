# COMP0250_CW2

This project was submitted as part of the Robotics and AI MSc at UCL. The overall framework within the src folder was provided by the university. My contributions can be found in the following folders, as well as the modification of miscellaneous files such as the CMakeLists.txt for the inclusion of various libraries:

src/obj_sort/src

src/obj_sort/include

Within these files main.cpp is responsible for handling the main setup and interfacing with ROS, BaseTask.cpp contains a variety of functions used throughout the project, and the files Task1.cpp, Task2.cpp and Task3.cpp are responsible for providing the framework for each task. These tasks work primarily off inputs provided by ROS services, which provided key information needed for the completion of each task.

Task 1: Simple Pick and Place
Given a pick and a place position pick up and deposit an object in the provided location. This was a farily simple task where motion planning was accomplished through the MoveIt interface

Task 2: Simple Shape Matching
Given the location of a reference object and a test object, identify if the target object is an instance of the reference object 

Task 3: Combined Shape Matching/ Pick and Place
This was a much more complex task which took in no information. The entire board needed to be scanned with the RGB-D camera, after which the point cloud needed to be clustered to identify the number of shapes. Various outlier removal methods were used, the shapes were matched and then an instance of the most common shape was placed into the goal point which also needed to be identified.

