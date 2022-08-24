#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs//Odometry.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include "livox_ros_driver/CustomMsg.h"
#include "livox_ros_driver/CustomPoint.h"
#include "Eigen/Dense"
#include "rosbag_to_2000/wgs84_do_puwg92.h"
#include "algorithm"
#define MAX_NUMBER_POINTS 1000000

template<typename T>
inline bool save_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
    std::ofstream ofs(file_name, std::ios::binary);
    if (!ofs.good()) {
        return false;
    }
    ofs.write(reinterpret_cast<char*>(vector_data.data()), vector_data.size()* sizeof(T));
    return true;
}

namespace structs {
    struct Point {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        uint32_t intensity = 0;
        double time = 0.0;
    };
    struct ChunkFile {
        int filename;
        double time_begin_inclusive;
        double time_end_inclusive;
    };
}
int main()
{
    const std::string fn {"/media/michal/ext/orto_photo_skierniewice/hd_skierniewice/2022-08-19T13:05:12.524438/"};
    const std::string out_dir_name {fn+"/data5"};
    boost::filesystem::create_directory(out_dir_name);
    using namespace boost::filesystem;
    recursive_directory_iterator dir_it ( fn );

    std::vector<std::string> patches;
    while (dir_it!=recursive_directory_iterator{})
    {
        if (dir_it->path().extension() == ".bag"){
            patches.push_back(dir_it->path().string());
        }
        dir_it++;
    }
    std::sort(patches.begin(),patches.end());

    std::ofstream trajectory_file(out_dir_name+"/odometry.csv");
    std::ofstream gnss_file(out_dir_name+"/gnss.csv");
    trajectory_file<<"timestamp;t0;t1;t2;r00;r01;r02;r10;r11;r12;r20;r21;r22"<<std::endl;
    gnss_file<<"timestamp;t0;t1;t2;r00;r01;r02;r10;r11;r12;r20;r21;r22"<<std::endl;


    std::vector<structs::ChunkFile> chunk_files;
    std::vector<structs::Point> points;
    ros::Time last_time(0);
    for (const auto &p : patches){
        std::cout << "processing " << p << std::endl;
        rosbag::Bag bag;
        bag.open(p);  // BagMode is Read by default
        std::vector<std::string> topics;
        topics.push_back(std::string("/livox/lidar"));
        topics.push_back(std::string("/Odometry"));
        topics.push_back(std::string("/fix"));


        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(rosbag::MessageInstance const m: view) {

            livox_ros_driver::CustomMsgConstPtr  cptr = m.instantiate<livox_ros_driver::CustomMsg>();
            if (cptr) {
                assert(cptr->header.stamp>=last_time);
                last_time = cptr->header.stamp;
                for (const auto &pp : cptr->points) {
                    structs::Point cp;
                    cp.x = pp.x;
                    cp.y = pp.y;
                    cp.z = pp.z;
                    cp.intensity = pp.reflectivity;
                    cp.time = cptr->header.stamp.toSec() + static_cast<double>(pp.offset_time)/1e9;
                    points.push_back(cp);
                }

                if (points.size() >= MAX_NUMBER_POINTS) {

                    structs::ChunkFile chunkfile;
                    chunkfile.filename = (int) chunk_files.size();
                    chunkfile.time_begin_inclusive = points[0].time;
                    chunkfile.time_end_inclusive = points[points.size() - 1].time;
                    chunk_files.push_back(chunkfile);

                    std::string filename = out_dir_name + "/" + std::to_string(chunkfile.filename) + ".bin";
                    save_vector_data<structs::Point>(filename, points);

                    std::cout << std::setprecision(20);
                    std::cout << "saving to file " << filename << " nr points " << points.size() << " "
                              << points[0].time << " " << points[points.size() - 1].time << std::endl;

                    filename = out_dir_name + "/" + "chunks.bin";
                    save_vector_data<structs::ChunkFile>(filename, chunk_files);

                    points.clear();
                    points.reserve(MAX_NUMBER_POINTS);
                }
            }
            nav_msgs::OdometryConstPtr codom = m.instantiate<nav_msgs::Odometry>();
            if (codom)
            {
                trajectory_file << codom->header.stamp <<";";
                trajectory_file << codom->pose.pose.position.x <<";";
                trajectory_file << codom->pose.pose.position.y <<";";
                trajectory_file << codom->pose.pose.position.z <<";";
                const auto& msg_o = codom->pose.pose.orientation;
                Eigen::Quaterniond quat (msg_o.w,msg_o.x,msg_o.y,msg_o.z);
                Eigen::Matrix3d mat = quat.toRotationMatrix();
                trajectory_file << mat(0,0) <<";";
                trajectory_file << mat(0,1) <<";";
                trajectory_file << mat(0,2) <<";";
                trajectory_file << mat(1,0) <<";";
                trajectory_file << mat(1,1) <<";";
                trajectory_file << mat(1,2) <<";";
                trajectory_file << mat(2,0) <<";";
                trajectory_file << mat(2,1) <<";";
                trajectory_file << mat(2,2) <<"\n";
            }
            sensor_msgs::NavSatFixConstPtr cfix = m.instantiate<sensor_msgs::NavSatFix>();
            if (cfix)
            {
                double L = cfix->longitude;
                double B = cfix->latitude;

                std::array<double, 2> xy;
                wgs84_do_puwg92(B, L, xy.data(), xy.data() + 1);
                gnss_file << last_time<<";";
                gnss_file << xy[1]<<";";
                gnss_file << xy[0]<<";";
                gnss_file << cfix->altitude <<";";
                Eigen::Matrix3d mat =Eigen::Matrix3d::Identity();
                gnss_file << mat(0,0) <<";";
                gnss_file << mat(0,1) <<";";
                gnss_file << mat(0,2) <<";";
                gnss_file << mat(1,0) <<";";
                gnss_file << mat(1,1) <<";";
                gnss_file << mat(1,2) <<";";
                gnss_file << mat(2,0) <<";";
                gnss_file << mat(2,1) <<";";
                gnss_file << mat(2,2) <<"\n";
            }
        }

        bag.close();
    }





}