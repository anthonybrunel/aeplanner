#include "polytrajinterface.h"



PolyTrajInterface::PolyTrajInterface(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
{
    publish_timer_ = nh_.createTimer(ros::Duration(0.01),
                                     &PolyTrajInterface::commandTimerCallback,
                                     this, false, false);

    vmax_=2.5;
    amax_=3.;
    nh_private_.param("rpl_exploration_rotors/vmax", vmax_, vmax_);
    nh_private_.param("rpl_exploration_rotors/amax", amax_, amax_);

    std::cout << "[PolyTrajInterface] Initialized" <<std::endl;
    std::cout << "max acc: " << amax_ << "max vel: " << vmax_ <<std::endl;
    pub_markers_ =
            nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1000);


    command_pub_ = nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

}

bool PolyTrajInterface::computeTrajectory(const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel, mav_trajectory_generation::Trajectory *trajectory)
{
    std::cout << "New Trajectory " << start_pos.transpose() << " "<< start_vel.transpose() << " " << goal_pos .transpose()<< " " << goal_vel.transpose() <<std::endl;

    const int dimension = goal_pos.size();
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;


    std::vector<Eigen::Vector4d> pts;
    //    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,start_pos);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel);
    vertices.push_back(start);

    mav_trajectory_generation::Vertex p(dimension);
    //    for(size_t i = 1; i < pts.size()-1; ++i){

    //        Eigen::Vector4d &next_pts = pts[i];
    //        p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,next_pts);
    ////        p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
    ////                          Eigen::Vector4d(0,0,0.3,0));


    //        vertices.push_back(p);

    //    }
    //    Eigen::Vector4d half_path;half_path << (goal_pos.x() - start_pos_4d.x())/2.,(goal_pos.y() - start_pos_4d.y())/2.,(goal_pos.z() - start_pos_4d.z())/2.,M_PI/2.;
    //    p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,half_path);
    //    vertices.push_back(p);


    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      Eigen::Vector4d(0,0,0,0));

    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, 2, 2.);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    // set up optimization problem
    const int N = 8;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vmax_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, amax_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(vmax_, amax_);

    return true;
}

bool PolyTrajInterface::computeTrajectory(const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                                          const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> &wps,
                                          mav_trajectory_generation::Trajectory *trajectory)
{

    std::cout << "New Trajectory with waypoints" << start_pos.transpose() << " "<< start_vel.transpose() << " " << goal_pos .transpose()<< " " << goal_vel.transpose() <<std::endl;

    const int dimension = goal_pos.size();
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;


    //    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,start_pos);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel);
    vertices.push_back(start);

    mav_trajectory_generation::Vertex p(dimension);
    for(size_t i = 0; i < wps.size(); ++i){
        const Eigen::Vector4d &next_pts = wps[i];
        p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,next_pts);
        p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            Eigen::Vector4d::Zero());
        p.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                          Eigen::Vector4d(0,0,0,0));
        vertices.push_back(p);
    }


    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      Eigen::Vector4d(0,0,0,0));

    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, 1.5, 2.);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 2000.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vmax_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, amax_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(vmax_, amax_);

    return true;

}


void PolyTrajInterface::commandTimerCallback(const ros::TimerEvent &)
{
    trajectory_mtx_.lock();
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    if (current_sample_time_ <= trajectory_.getMaxTime()) {
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                    trajectory_, current_sample_time_, &trajectory_point);
        if (!success) {
            publish_timer_.stop();
        }

        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
        command_pub_.publish(msg);
    } else if(current_sample_time_ <= trajectory_.getMaxTime() + 1.){//time shift to go at goal
        current_sample_time_ = trajectory_.getMaxTime();
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                    trajectory_, current_sample_time_, &trajectory_point);
        if (!success) {
            publish_timer_.stop();
        }
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    }else {
        publish_timer_.stop();
        trajectory_.clear();
    }
    trajectory_mtx_.unlock();
}

void PolyTrajInterface::startTrajectory()
{
    if(trajectory_.getMaxTime() <= 0)
        return;

    publish_timer_.start();
    current_sample_time_ = 0.0;
    start_time_ = ros::Time::now();

}

bool PolyTrajInterface::isRunning()
{
    return publish_timer_.hasStarted();
}

void PolyTrajInterface::setTrajectory(const mav_trajectory_generation::Trajectory &trajectory)
{
    trajectory_mtx_.lock();
    publish_timer_.stop();
    trajectory_.clear();
    trajectory_ = trajectory;
    trajectory_mtx_.unlock();

}




void PolyTrajInterface::publishVizualization(const mav_trajectory_generation::Trajectory &trajectory)
{
    if(trajectory.empty())
        return;
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance =
            0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";


    mav_trajectory_generation::drawMavTrajectory(trajectory,
                                                 distance,
                                                 frame_id,
                                                 &markers);
    pub_markers_.publish(markers);
}
