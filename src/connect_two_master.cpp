#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <boost/function.hpp>

int shmid;

enum{ LLEG,RLEG,LARM,RARM,COM,HEAD,NUM_TGTS};
enum{ LF,RF,LH,RH,NUM_EES};
std::vector<std::string> ee_names, tgt_names;

#define MAX_FRAME_ID_SIZE 1024
#define DP1 std::fixed << std::setprecision(1)
const int RATE = 1000;

struct StaticHeader{
    char frame_id[MAX_FRAME_ID_SIZE];// header.frame_id is varid length and unsuitable for shm
    uint32_t seq;
    ros::Time stamp;
};
struct StaticPoseStamed{
    StaticHeader header;
    geometry_msgs::Pose pose;
};
struct StaticWrenchStamped{
    StaticHeader header;
    geometry_msgs::Wrench wrench;
};
struct ros_shm_t{
    StaticPoseStamed masterTgtPoses[NUM_TGTS];
    StaticWrenchStamped slaveEEWrenches[NUM_EES];
};

inline std::ostream& operator<<(std::ostream& os, geometry_msgs::PoseStamped& in){
    os << std::fixed << std::setprecision(2) << "= "
        << in.pose.position.x << ", " << in.pose.position.y << ", " << in.pose.position.z << ", "
        << in.pose.orientation.x << ", " << in.pose.orientation.y << ", " << in.pose.orientation.z << ", " << in.pose.orientation.w
        << " [ " << in.header.seq << ": " << in.header.frame_id <<"]";
    return os;
}
inline std::ostream& operator<<(std::ostream& os, geometry_msgs::WrenchStamped& in){
    os << std::fixed << std::setprecision(2) << "= "
        << in.wrench.force.x << ", " << in.wrench.force.y << ", " << in.wrench.force.z << ", "
        << in.wrench.torque.x << ", " << in.wrench.torque.y << ", " << in.wrench.torque.z << ", "
        << " [ " << in.header.seq << ": " << in.header.frame_id <<"]";
    return os;
}

void onMasterTgtPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, StaticPoseStamed* ret_ptr){
    msg->header.frame_id.copy(ret_ptr->header.frame_id, MAX_FRAME_ID_SIZE);
    ret_ptr->header.seq = msg->header.seq;
    ret_ptr->header.stamp = msg->header.stamp;
    ret_ptr->pose = msg->pose;
}

void process1(int argc, char** argv) {
    putenv("ROS_MASTER_URI=http://tablis:11311");
    unsigned long loop = 0;
    ros_shm_t* shmaddr;
    ros::init(argc, argv, "process1");
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    std::string pname = "process1";
    ROS_INFO_STREAM(pname << " start");
    std::vector<ros::Subscriber> masterTgtPoses_sub;
    std::vector<ros::Publisher> slaveEEWrenches_pub;

    if ((shmaddr = (ros_shm_t*)shmat(shmid, NULL, 0)) == (void *) -1) {
        ROS_ERROR_STREAM(pname << " shmat error");
        exit(EXIT_FAILURE);
    }else{
        ROS_INFO_STREAM(pname << " shmat success");
    }

    for(int i=0; i<tgt_names.size(); i++){
        std::string topic = "master_"+tgt_names[i]+"_pose_out";
        ROS_INFO_STREAM(pname << " register subscriber " << topic);
        masterTgtPoses_sub.push_back( n.subscribe<geometry_msgs::PoseStamped>(topic, 1,
            boost::bind(onMasterTgtPoseCB, _1, &shmaddr->masterTgtPoses[i])));
    }
    for(int i=0; i<ee_names.size(); i++){
        std::string topic = "slave_"+ee_names[i]+"_wrench_in";
        ROS_INFO_STREAM(pname << " register publisher " << topic);
        slaveEEWrenches_pub.push_back( n.advertise<geometry_msgs::WrenchStamped>(topic, 1));
    }

    std::vector<geometry_msgs::WrenchStamped> tmp(ee_names.size());
    while (ros::ok()) {
        for(int i=0; i<ee_names.size(); i++){
            if(tmp[i].header.seq == shmaddr->slaveEEWrenches[i].header.seq){ continue; } // skip if same data
            tmp[i].header.frame_id  = shmaddr->slaveEEWrenches[i].header.frame_id;
            tmp[i].header.stamp     = shmaddr->slaveEEWrenches[i].header.stamp;
            tmp[i].header.seq       = shmaddr->slaveEEWrenches[i].header.seq;
            tmp[i].wrench           = shmaddr->slaveEEWrenches[i].wrench;
            slaveEEWrenches_pub[i].publish(tmp[i]);
            if(loop%RATE == 0){
                ROS_INFO_STREAM("Wrench:"<<ee_names[i]<< tmp[i] );
            }
        }
        ros::spinOnce();
        rate.sleep();
        loop++;
    }

    if (shmdt(shmaddr) == 0) {
        ROS_INFO("process1: shmdt success");
    }else{
        ROS_ERROR("process1: shmdt fail"); exit(EXIT_FAILURE);
    }
    ROS_INFO("process1: EXIT_SUCCESS");
    exit(EXIT_SUCCESS);
}

void onslaveEEWrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg, StaticWrenchStamped* ret_ptr){
    msg->header.frame_id.copy(ret_ptr->header.frame_id, MAX_FRAME_ID_SIZE);
    ret_ptr->header.seq = msg->header.seq;
    ret_ptr->header.stamp = msg->header.stamp;
    ret_ptr->wrench = msg->wrench;
}

void process2(int argc, char** argv) {
    putenv("ROS_MASTER_URI=http://jaxonred:11311");
    unsigned long loop = 0;
    ros_shm_t* shmaddr;
    ros::init(argc, argv, "process2");
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    std::string pname = "process2";
    ROS_INFO_STREAM(pname << " start");

    std::vector<ros::Publisher> masterTgtPoses_pub;
    std::vector<ros::Subscriber> slaveEEWrenches_sub;

    sleep(1);

    if ((shmaddr = (ros_shm_t*)shmat(shmid, NULL, 0)) == (void *) -1) {
        ROS_ERROR_STREAM(pname << " shmat error");
        exit(EXIT_FAILURE);
    }else{
        ROS_INFO_STREAM(pname << " shmat success");
    }

    for(int i=0; i<tgt_names.size(); i++){
        std::string topic = "master_"+tgt_names[i]+"_pose_in";
        ROS_INFO_STREAM(pname << " register publisher " << topic);
        masterTgtPoses_pub.push_back( n.advertise<geometry_msgs::PoseStamped>(topic, 1));
    }
    for(int i=0; i<ee_names.size(); i++){
        std::string topic = "slave_"+ee_names[i]+"_wrench_out";
        ROS_INFO_STREAM(pname << " register subscriber " << topic);
        slaveEEWrenches_sub.push_back( n.subscribe<geometry_msgs::WrenchStamped>(topic, 1,
            boost::bind(onslaveEEWrenchCB, _1, &shmaddr->slaveEEWrenches[i])));
    }

    std::vector<geometry_msgs::PoseStamped> tmp(tgt_names.size());
    while (ros::ok()) {
        for(int i=0; i<tgt_names.size(); i++){
            if(tmp[i].header.seq == shmaddr->masterTgtPoses[i].header.seq){ continue; } // skip if same data
            tmp[i].header.frame_id = shmaddr->masterTgtPoses[i].header.frame_id;
            tmp[i].header.stamp    = shmaddr->masterTgtPoses[i].header.stamp;
            tmp[i].header.seq      = shmaddr->masterTgtPoses[i].header.seq;
            tmp[i].pose            = shmaddr->masterTgtPoses[i].pose;
            masterTgtPoses_pub[i].publish(tmp[i]);
            if(loop%RATE == 0){
                ROS_INFO_STREAM("Pose:"<<tgt_names[i]<< tmp[i] );
            }
        }
        ros::spinOnce();
        rate.sleep();
        loop++;
    }

    if (shmdt(shmaddr) == 0) {
        ROS_INFO_STREAM(pname << " shmdt success");
    }else{
        ROS_ERROR_STREAM(pname << " shmdt fail"); exit(EXIT_FAILURE);
    }
    ROS_INFO_STREAM(pname << " EXIT_SUCCESS");
    exit(EXIT_SUCCESS);
}





int main(int argc, char** argv) {
    int child_cnt;

    std::cout << std::fixed;
    std::cout << std::setprecision(1);

    ee_names.push_back("lleg");
    ee_names.push_back("rleg");
    ee_names.push_back("larm");
    ee_names.push_back("rarm");
    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");

    if ((shmid = shmget(IPC_PRIVATE, sizeof(ros_shm_t), 0600)) == -1) {
        perror("main : shmget ");
        exit(EXIT_FAILURE);
    }

    if(fork() == 0){ process1(argc, argv); }
    if(fork() == 0){ process2(argc, argv); }

    for (child_cnt = 0; child_cnt < 2; ++child_cnt) {
        wait(NULL);
    }
    if (shmctl(shmid, IPC_RMID, NULL) == -1) {
        perror("main : shmctl ");
        exit(EXIT_FAILURE);
    }

    printf("親プロセス終了\n");
    return EXIT_SUCCESS;
}
