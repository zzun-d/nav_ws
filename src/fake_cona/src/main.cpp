#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_cona");
    ros::NodeHandle nh;
    
    //*******************************************************************************************************************

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", buffer);
    planner_costmap_ros_->pause();

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    // controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", buffer);
    // controller_costmap_ros_->pause();

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    // controller_costmap_ros_->start();

    // //if we shutdown our costmaps when we're deactivated... we'll do that now
    // if(shutdown_costmaps_){
    //   ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
    //   planner_costmap_ros_->stop();
    //   controller_costmap_ros_->stop();
    // }



    //*******************************************************************************************************************

    ros::Rate rate(1);
    while(ros::ok)
    {
        ROS_INFO("hi");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}