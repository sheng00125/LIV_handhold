#include <iostream>
#include <fstream>
#include <string>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;
using namespace Eigen;
double secs_init, nsecs_init, last_time;
bool is_img, is_init = false;
string bag_path, write_path, topic_name;

void parse_time() 
{
    fstream file_;
    file_.open(bag_path, ios::in);
    if (!file_)
    {
        cout << "File " << bag_path << " does not exit" << endl;
        return;
    }
    
    rosbag::Bag bag;
    try
    {
        ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
        bag.open(bag_path, rosbag::bagmode::Read);
        ROS_INFO("Bag %s opened", bag_path.c_str());
    }
    catch (rosbag::BagException e)
    {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> topics;
    if (is_img)
        topics.push_back(topic_name);
    else
        topics.push_back(string("/livox/lidar"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ofstream file_w;
    file_w.open(write_path, std::ofstream::trunc);
    for (const rosbag::MessageInstance& m : view)
    {
        double secs, nsecs, inc;
        if (is_img)
        {
            auto msg = *(m.instantiate<sensor_msgs::Image>());
            secs = msg.header.stamp.sec;
            nsecs = msg.header.stamp.nsec;
        }
        else
        {
            auto msg = *(m.instantiate<sensor_msgs::PointCloud2>());
            secs = msg.header.stamp.sec;
            nsecs = msg.header.stamp.nsec;
        }

        if (!is_init)
        {
            secs_init = secs;
            nsecs_init = nsecs;
            is_init = true;
            secs -= secs_init;
            nsecs -= nsecs_init;
            last_time = secs + nsecs * 1e-9;
        }
        else
        {
            secs -= secs_init;
            nsecs -= nsecs_init;
            inc = secs + nsecs * 1e-9 - last_time;
            last_time = secs + nsecs * 1e-9;
            file_w << inc * 1e3 << "\n";
        }
    }
    file_w.close();
    cout<<"complete"<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parse_topic");
    ros::NodeHandle nh("~");

    nh.getParam("bag_path", bag_path);
    nh.getParam("write_path", write_path);
    nh.getParam("topic_name", topic_name);
    nh.getParam("is_img_topic", is_img);
    
    parse_time();
}