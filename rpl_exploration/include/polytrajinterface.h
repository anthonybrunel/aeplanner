#pragma once
#include <Eigen/Core>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mutex>


class PolyTrajInterface
{
public:
    PolyTrajInterface(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);


    bool computeTrajectory(const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,
                           mav_trajectory_generation::Trajectory* trajectory);
    bool computeTrajectory(const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                                              const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> &wps,
                                              mav_trajectory_generation::Trajectory *trajectory);

    void commandTimerCallback(const ros::TimerEvent&);

    void publishVizualization(const mav_trajectory_generation::Trajectory &trajectory);

    void startTrajectory();


    bool isRunning();





    void setTrajectory(const mav_trajectory_generation::Trajectory &trajectory);



    mav_trajectory_generation::Trajectory trajectory_;



    std::mutex trajectory_mtx_;


    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher command_pub_;
    ros::Publisher pub_markers_;

    ros::Timer publish_timer_;
    ros::Time start_time_;

    double current_sample_time_;

    float vmax_ = 1.5;
    float amax_ = 2.5;
};
