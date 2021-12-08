#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>

#include <aeplanner/Node.h>
#include <aeplanner/aeplannerAction.h>
#include <rrtplanner/rrtAction.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/Odometry.h>

#include <eigen_conversions/eigen_msg.h>
#include "flybo_utils/plannerlogging.h"
#include "flybo_utils/poly_trajectory_rpg.h"

enum State{
    NOTREADY,
    TAKEOFF,
    TRAJ,
    IDLE
};
bool odomReady = false;

Eigen::Affine3d pose;
Eigen::Vector3d vel;

State state = NOTREADY;
ros::Publisher saved_paths_pub_;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> saved_path_ompl_;
PlannerLogging planner_logger_;


void publishTrajectory(){
    visualization_msgs::MarkerArray markers_paths;
    if(saved_path_ompl_.size() == 0){
        return;
    }

    float max_r = 20/255.,max_g = 70/255.,max_b = 1.;
    float step_r = (1.-max_r)/(((float)saved_path_ompl_.size())-1.),
            step_g = (1.-max_g)/(((float)saved_path_ompl_.size())-1.),
            step_b = (1.-max_b)/(((float)saved_path_ompl_.size())-1.);






    geometry_msgs::Point pt;
    //    mk.action  = visualization_msgs::Marker::DELETE;

    //    for(int j = 1; j < saved_path_ompl_.size(); j+=1){

    //        mk.id              =  j-1;
    //        markers_paths.markers.push_back(mk);
    //    }
    float cpt= 0;
    for(int j = 1; j < saved_path_ompl_.size(); j+=1){

        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp    = ros::Time::now();
        mk.type            = visualization_msgs::Marker::LINE_STRIP;
        mk.action             = visualization_msgs::Marker::ADD;


        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

//        mk.color.r = 0./255.;
//        mk.color.g = 51/255.;
//        mk.color.b = 102/255.;

        mk.color.r = 255./255.;
        mk.color.g = 51/255.;
        mk.color.b = 60/255.;

        mk.color.a = 1;
        mk.scale.x = 0.15;

//        mk.color.r = 1.-step_r*cpt;
//        mk.color.g = 1.-step_g*cpt;

//        mk.color.b = 1.-step_b*cpt;
        cpt += 1;
        pt.x = saved_path_ompl_[j-1].x();
        pt.y = saved_path_ompl_[j-1].y();
        pt.z = saved_path_ompl_[j-1].z();
        mk.points.push_back(pt);

        pt.x = saved_path_ompl_[j].x()    ;
        pt.y = saved_path_ompl_[j].y();
        pt.z = saved_path_ompl_[j].z();
        mk.points.push_back(pt);
        mk.id              =  j-1;

        markers_paths.markers.push_back(mk);
    }


    saved_paths_pub_.publish(markers_paths);
}
std::vector<double> times;
void printTimeComparison()
{
}

void uavOdomCallback(const nav_msgs::OdometryConstPtr &odom_i){
    odomReady =true;

    tf::poseMsgToEigen(odom_i->pose.pose, pose);
    tf::vectorMsgToEigen(odom_i->twist.twist.linear, vel);

    double speed = vel.norm();
    planner_logger_.log_dist_speed(speed,pose.translation());

}


void sendGoal(PolyTrajInterface &traj_interface,const Eigen::Vector4d &start, const Eigen::Vector4d &vel_start, const Eigen::Vector4d &goal){
    mav_trajectory_generation::Trajectory traj;
    traj_interface.computeTrajectoryAndYaw(start,vel_start,goal,Eigen::Vector4d::Zero(),std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >()
                                           ,&traj);
    traj_interface.setTrajectory(traj);
    traj_interface.startTrajectory();
//    traj_interface.sendTrajectory();
    state = TRAJ;
}
void sendGoal(PolyTrajInterface &traj_interface,const Eigen::Vector4d &start, const Eigen::Vector4d &vel_start, const Eigen::Vector4d &goal,
              const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps){
    mav_trajectory_generation::Trajectory traj;
    traj_interface.computeTrajectoryAndYaw(start,vel_start,goal,Eigen::Vector4d::Zero(),wps,&traj);
    traj_interface.setTrajectory(traj);
    traj_interface.startTrajectory();
//    traj_interface.sendTrajectory();

    state = TRAJ;
}


void correctYaw(Eigen::Vector4d& start,Eigen::Vector4d& eig_goal){
    if (start.w() < 0) { start.w() += 2 * M_PI; }
    //    if (eig_goal.w() < 0) { eig_goal.w() += 2 * M_PI; }

    if(std::fabs(eig_goal.w()-start.w()) > M_PI){
        if(start.w() < eig_goal.w()){
            start.w() += M_PI*2;
        }else{
            eig_goal.w() += 2 * M_PI;
        }
    }
}

double angleDifference(double angle1, double angle2) {
  double angle = std::abs(std::fmod(angle1 - angle2, 2.0 * M_PI));
  if (angle > M_PI) {
    angle = 2.0 * M_PI - angle;
  }
  return angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpl_exploration_rotors");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");


    ROS_INFO("Started exploration");
    ros::Subscriber sub_odom_     =
            nh_private.subscribe<nav_msgs::Odometry>("uav_odom", 10, uavOdomCallback);

    // Open logfile;
    std::string path = ros::package::getPath("rpl_exploration");
    std::ofstream logfile, pathfile;
    logfile.open(path + "/data/logfile.csv");
    pathfile.open(path + "/data/path.csv");
    std::string log_path_ ="";
    nh_private.param("planner/save_log_folder", log_path_, log_path_);

    saved_paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/aep/saved_path", 10);



    // wait for aep server to start
    ROS_INFO("Waiting for aeplanner action server");
    actionlib::SimpleActionClient<aeplanner::aeplannerAction> aep_ac("make_plan",
                                                                     true);
    aep_ac.waitForServer();  // will wait for infinite time
    ROS_INFO("aeplanner action server started!");


    // wait for fly_to server to start
    ROS_INFO("Waiting for rrt action server");
    actionlib::SimpleActionClient<rrtplanner::rrtAction> rrt_ac("rrt", true);
    // rrt_ac.waitForServer(); //will wait for infinite time
    ROS_INFO("rrt Action server started!");

    PolyTrajInterface traj_interface(nh,nh_private);
    ros::Rate loop(100);

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> waypoints;
    Eigen::Vector4d start;
    Eigen::Vector4d vel_start;

    int iteration = 0;
    int actions_taken = 1;


    ros::Time start_time = ros::Time::now();
    geometry_msgs::PoseStamped last_pose;
    ros::Duration fly_time;
    ros::Time s(0);
    ros::Duration(5.0).sleep();
    Timer t;
    Timer t_iter;
    double dist = 0;
    float max_t = 1300;
    Eigen::Vector3d target;
    double target_yaw;
    while(ros::ok()){
        switch(state){
        case NOTREADY:
            std::cout << "Planner waiting for odometry" << std::endl;
            if(odomReady)
                state = TAKEOFF;

            break;
        case TAKEOFF:
            std::cout << "Planner rotate in place" << std::endl;

            ROS_INFO("Starting the planner: Performing initialization motion");
            start << pose.translation(),mav_msgs::yawFromQuaternion(
                        (Eigen::Quaterniond)pose.rotation());
            target = start.head(3);
            target_yaw = start.w()+M_PI*1.5;

            waypoints.push_back(Eigen::Vector4d(start.x(),start.y(),start.z(),start.w()+M_PI*.75));
            sendGoal(traj_interface,start,Eigen::Vector4d::Zero(),Eigen::Vector4d(start.x(),start.y(),start.z(),start.w()+M_PI*1.5),waypoints);
            waypoints.clear();
            t.restart();
            break;
        case TRAJ:
//            if ((target - pose.translation()).norm() <
//                0.15) {
//                // check goal yaw reached (if tol is set)

//                Eigen::Quaterniond qe(pose.rotation());
//                tf::Quaternion q(
//                            qe.x(),
//                            qe.y(),
//                            qe.z(),
//                            qe.w()

//                            );
//                double yaw = tf::getYaw(q);

//                if (angleDifference(target_yaw, yaw) <
//                    0.05) {
//                    state=IDLE;

//                }
//            }
//            break;
            if(!traj_interface.isRunning()){
                if(s == ros::Time(0)){

                }else{
                    fly_time = ros::Time::now() - s;
                    ros::Duration elapsed = ros::Time::now() - start_time;

                    ROS_INFO_STREAM("Iteration: "       << iteration << "  " <<
                                    "Time: "            << elapsed << "  " <<
                                    "Sampling: "        << aep_ac.getResult()->sampling_time.data << "  " <<
                                    "Planning: "        << aep_ac.getResult()->planning_time.data << "  " <<
                                    "Collision check: " << aep_ac.getResult()->collision_check_time.data  << "  " <<
                                    "Flying: "          << fly_time << " " <<
                                    "Tree size: "       << aep_ac.getResult()->tree_size);

                    logfile << iteration << ", "
                            << elapsed << ", "
                            << aep_ac.getResult()->sampling_time.data << ", "
                            << aep_ac.getResult()->planning_time.data << ", "
                            << aep_ac.getResult()->collision_check_time.data << ", "
                            << fly_time << ", "
                            << aep_ac.getResult()->tree_size << std::endl;

                    double std_times= 0;
                    std_times = 0;
                    float mean = 0;
                    for(size_t i = 0; i < times.size(); ++i){
                        mean += times[i];
                    }
                    mean /= (double)times.size();
                    for(size_t i = 0; i < times.size(); ++i){
                        std_times += pow(times[i] - mean, 2);
                    }
                    std_times /= (double)times.size();
                    std_times = sqrt(std_times);


                    std::cout<< "distance " << dist<<std::endl;


                    iteration++;
                }


                state=IDLE;
                break;
            }
            break;
        case IDLE:

            ROS_INFO_STREAM("Planning iteration " << iteration);
            t_iter.restart();
            aeplanner::aeplannerGoal aep_goal;
            aep_goal.header.stamp = ros::Time::now();
            aep_goal.header.seq = iteration;
            aep_goal.header.frame_id = "map";
            aep_goal.actions_taken = actions_taken;
            aep_ac.sendGoal(aep_goal);
            while (!aep_ac.waitForResult(ros::Duration(0.01)))
            {
                if(!nh.ok()){
                    break;
                }
                ros::spinOnce();
            }
            times.push_back(t_iter.elapsed_ms());

            if (aep_ac.getResult()->is_clear)
            {

                planner_logger_.addTime(t_iter.elapsed_ms());
                actions_taken = 0;

                s = ros::Time::now();
                geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;

                // Write path to file
                pathfile << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
                         << ", " << goal_pose.pose.position.z << ", n" << std::endl;

                last_pose.pose = goal_pose.pose;
                Eigen::Vector4d eig_goal;
                Eigen::Quaterniond q(goal_pose.pose.orientation.w,
                                     goal_pose.pose.orientation.x,
                                     goal_pose.pose.orientation.y,
                                     goal_pose.pose.orientation.z);
                eig_goal << goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z,mav_msgs::yawFromQuaternion(
                            (Eigen::Quaterniond)q);



                start << pose.translation(),mav_msgs::yawFromQuaternion(
                            (Eigen::Quaterniond)pose.rotation());

                vel_start<<vel.x(),vel.y(),vel.z(),0;
                if(saved_path_ompl_.size() == 0){
                    saved_path_ompl_.push_back(Eigen::Vector3d(start.x(),start.y(),start.z()));
                }
                dist += (start.head(3)-eig_goal.head(3)).norm();
                saved_path_ompl_.push_back(Eigen::Vector3d(eig_goal.x(),eig_goal.y(),eig_goal.z()));

                target = eig_goal.head(3);
                target_yaw = eig_goal(3);
                sendGoal(traj_interface,start,vel_start,eig_goal);
                std::cout << "AEP plan" <<std::endl;

            }
            else
            {
                rrtplanner::rrtGoal rrt_goal;
                rrt_goal.start.header.stamp = ros::Time::now();
                rrt_goal.start.header.frame_id = "map";
                tf::poseEigenToMsg(pose,rrt_goal.start.pose);
                if (!aep_ac.getResult()->frontiers.poses.size())
                {
                    ROS_WARN("Exploration complete!");
                    break;
                }
                for (auto it = aep_ac.getResult()->frontiers.poses.begin();
                     it != aep_ac.getResult()->frontiers.poses.end(); ++it)
                {
                    rrt_goal.goal_poses.poses.push_back(*it);
                }


                rrt_ac.sendGoal(rrt_goal);
                while (!rrt_ac.waitForResult(ros::Duration(0.01)))
                {
                    if(!nh.ok()){
                        break;
                    }
                    ros::spinOnce();
                }
                planner_logger_.addTime(t_iter.elapsed_ms());
                times.push_back(t_iter.elapsed_ms());

                nav_msgs::Path path = rrt_ac.getResult()->path;

                s = ros::Time::now();
                start << pose.translation(),mav_msgs::yawFromQuaternion(
                            (Eigen::Quaterniond)pose.rotation());
                Eigen::Vector4d eig_goal;
                vel_start<<vel.x(),vel.y(),vel.z(),0;
                if(path.poses.size() == 1){
                    std::cout << "wrong pose"<<std::endl;
                    s = ros::Time(0);
                    continue;
                }

                if(saved_path_ompl_.size() == 0){
                    saved_path_ompl_.push_back(Eigen::Vector3d(start.x(),start.y(),start.z()));
                }


                for (int i = path.poses.size() - 1; i >= 0; --i)
                {

                    geometry_msgs::Pose goal_pose = path.poses[i].pose;
                    // Write path to file
                    pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", "
                             << goal_pose.position.z << ", f" << std::endl;
                    last_pose.pose = goal_pose;

                    Eigen::Quaterniond q(goal_pose.orientation.w,
                                         goal_pose.orientation.x,
                                         goal_pose.orientation.y,
                                         goal_pose.orientation.z);
                    eig_goal << goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,mav_msgs::yawFromQuaternion(
                                (Eigen::Quaterniond)q);
                    saved_path_ompl_.push_back(eig_goal.head(3));
                    dist += (saved_path_ompl_[saved_path_ompl_.size()-2]-saved_path_ompl_[saved_path_ompl_.size()-1]).norm();



                    if(i == 0){
                        continue;
                    }else{

                        waypoints.push_back(eig_goal);
                    }
                }
                std::cout << "RRT plan" <<std::endl;

                target = eig_goal.head(3);
                target_yaw = eig_goal(3);

                sendGoal(traj_interface,start,vel_start,eig_goal,waypoints);
                waypoints.clear();
                actions_taken = -1;

            }
            publishTrajectory();
        }
        loop.sleep();
        ros::spinOnce();

    }
    planner_logger_.saveData(log_path_);
    return 0;
    //send 0,0,1.2
    //send 1,0,1.2
}
