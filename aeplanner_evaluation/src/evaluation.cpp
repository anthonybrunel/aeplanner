#include <ros/ros.h>
#include <signal.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "timer.hpp"
#include <fstream>
#include <iostream>
#include <ros/xmlrpc_manager.h>
#include <sys/types.h>
#include <dirent.h>

void octomapCallback(const octomap_msgs::Octomap& msg);
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

class CoverageEvaluator{
private:



    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::ServiceServer coverage_srv_;

    std::vector<std::pair<float,float> > scores_;
    double total_volume_;
    double coverage_, free_, occupied_, unmapped_;
    Timer t_;
    float delta_insertion_ = 1.f;

    double resolution_ =0.2;
    std::vector<double> min_;
    std::vector<double> max_;

public:
    CoverageEvaluator(ros::NodeHandle& nh) : nh_(nh),
        octomap_sub_(nh_.subscribe("octomap_full", 10, &CoverageEvaluator::octomapCallback2, this)){
        nh_.param("/boundary/eval/min", min_, std::vector<double>());
        nh_.param("/boundary/eval/max", max_, std::vector<double>());
        nh_.param("/octomap_server/resolution", resolution_, 0.2);
        if (!nh.getParam("volume", total_volume_)) {
            ROS_ERROR("No volume found...");
        }

        scores_.reserve(200000);
        t_.restart();
        scores_.push_back(std::pair<float,float>(0,0));
        ROS_INFO("Coverage evaluation init res: %F",resolution_);

    }
    ~CoverageEvaluator(){
        logExploration();
        std::cout << "AEP Evaluation finish correctly " <<std::endl;

    }
    void octomapCallback(const octomap_msgs::Octomap& msg);
    void octomapCallback2(const octomap_msgs::Octomap& msg);


    void logExploration();
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coverage_evaluation");
    ros::NodeHandle nh;

    {
        CoverageEvaluator ce_(nh);

        //    signal(SIGINT, mySigIntHandler);

        // Override XMLRPC shutdown

        // Create publishers, subscribers, etc.

        // Do our own spin loop
        ros::spin();

    }

    return 0;
}

void CoverageEvaluator::octomapCallback(const octomap_msgs::Octomap& msg){
    octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
    octomap::OcTree * ot = (octomap::OcTree*)aot;
    double volume = 0;

    for(octomap::OcTree::leaf_iterator it  = ot->begin_leafs();
        it != ot->end_leafs(); ++it){
        // if(!ot->isNodeOccupied(*it)){
        double l = it.getSize();
        volume += l*l*l;
        // }
    }

    coverage_ = volume;
}

void CoverageEvaluator::octomapCallback2(const octomap_msgs::Octomap& msg){

    try{


    octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
    if(!aot)
        return;
    octomap::OcTree * ot = (octomap::OcTree*)aot;
    if(!ot)
        return;
    double occupied = 0;
    double free     = 0;
    double unmapped = 0;

    double res = resolution_;
    double dV = res*res*res;
    if(t_.elapsed_s() < delta_insertion_){
        return;
    }

    for(double x = min_[0]; x <= max_[0]; x+=res){
        for(double y = min_[1]; y <= max_[1]; y+=res){
            for(double z = min_[2]; z <= max_[2]; z+=res){
                octomap::OcTreeNode* result = ot->search(x+res/2,y+res/2,z+res/2);
                if (!result)
                    unmapped+=dV;
                else if(result->getLogOdds() > 0)
                    occupied+=dV;
                else
                    free+=dV;
            }
        }
    }

    coverage_ = free+occupied;
    free_ = free;
    occupied_ = occupied;
    unmapped_ = unmapped;
    if(coverage_ > 5){
        scores_.push_back(std::pair<float,float>(scores_[scores_.size()-1].first+t_.elapsed_ms(),coverage_));
        t_.restart();

    }else{
        t_.restart();
    }

    delete aot;
    std::cout << "coverage : " << coverage_ <<std::endl;

    }catch (...) { std::cout << "default exception\n"; }


}




void CoverageEvaluator::logExploration()
{
  std::ofstream outdata; // outdata is like cin
  DIR *dp;
  int i = 0;
  struct dirent *ep;
  dp = opendir ("/home/anthony/ros_project/motion_planning/src/oscar/data/aep");

  if (dp != NULL)
  {
    while (ep = readdir (dp))
      i++;

    (void) closedir (dp);
  }
  else
    perror ("Couldn't open the directory");

	i-=2;

    std::string file = "/home/anthony/ros_project/motion_planning/src/oscar/data/aep/scores_aep";
	file+= "_"+std::to_string(i)+".txt";
    std::cout <<"Exploration log to file " << file<<std::endl;
    outdata.open(file.c_str()); // opens the file
    if( !outdata ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        return;
    }

    for (i=0; i<scores_.size(); ++i){
        outdata << scores_[i].first << " " << scores_[i].second << std::endl;
    }
    outdata.close();

}
