#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs//Odometry.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include "livox_ros_driver2/CustomMsg.h"
#include "livox_ros_driver2/CustomPoint.h"
#include "Eigen/Dense"
#include "rosbag_to_2000/wgs84_do_puwg92.h"
#include <boost/program_options.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>

#include "algorithm"
#include "utils.hpp"
constexpr int MAX_NUMBER_POINTS  = 100000;

void saveMat(const std::string& fn, const Eigen::Matrix4d& mat){
    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
    std::ofstream fmt_ofs (fn);
    fmt_ofs << mat.format(HeavyFmt);
    fmt_ofs.close();
}

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
namespace po = boost::program_options;

std::string savePLY(const std::string& out_dir_name,int number, const pcl::PointCloud<pcl::PointXYZI>& pointcloud){
    char buffer[1024];
    snprintf(buffer, 1024, "part_%04d.ply", number);
    pcl::PCLPointCloud2 msg;
    pcl::toPCLPointCloud2(pointcloud, msg);
    pcl::PLYWriter writer;
    writer.write (out_dir_name+"/"+buffer, msg, Eigen::Vector4f::Zero (),
                  Eigen::Quaternionf::Identity (), true, false);

    return buffer;
}

struct resso{
    std::string file_name;
    Eigen::Matrix4d mat;
};

int main(int ac, char** av)
{
    po::variables_map vm;
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
                ("help", "produce help message")
                ("dir", po::value<std::string>(), "directory to work")
                ("max_points", po::value<int>()->default_value(1e9), "max no points in one ply")
                ("max_distance", po::value<float>()->default_value(10), "max distance in one ply")
                ("max_range", po::value<float>()->default_value(500), "max range");
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);


        if(vm.count("help")>0 || vm.count("dir")==0){
            std::cout << desc<<std::endl;
            return 0;
        }
    }
    catch(const std::exception &e){
        std::cerr << e.what() << std::endl;
        return 0;
    }

    const std::string fn{vm["dir"].as<std::string>()};

    const std::string out_dir_name {fn+"/data_resso"};
    boost::filesystem::create_directory(out_dir_name);

    std::cout << "Processing " << fn << std::endl;
    using namespace boost::filesystem;
    recursive_directory_iterator dir_it(fn);
    std::vector<std::string> patches;
    while (dir_it!=recursive_directory_iterator{})
    {
        if (dir_it->path().extension() == ".bag"){
            patches.push_back(dir_it->path().string());
        }
        dir_it++;
    }
    std::sort(patches.begin(),patches.end());


    ros::Time last_time(0);
    std::map<double,Eigen::Matrix4d> trajectory;

    for (const auto &p : patches) {
        std::cout << "processing " << p << std::endl;
        rosbag::Bag bag;
        bag.open(p);  // BagMode is Read by default

        rosbag::View view_odom(bag, rosbag::TopicQuery(std::vector<std::string>({"/Odometry"})));
        for (rosbag::MessageInstance const m: view_odom) {
            nav_msgs::OdometryConstPtr codom = m.instantiate<nav_msgs::Odometry>();
            if (codom) {
                const auto &msg_o = codom->pose.pose.orientation;
                Eigen::Quaterniond quat(msg_o.w, msg_o.x, msg_o.y, msg_o.z);
                Eigen::Affine3d mat = Eigen::Affine3d::Identity();
                mat.rotate(quat);
                mat.translation() = Eigen::Vector3d{
                        codom->pose.pose.position.x,
                        codom->pose.pose.position.y,
                        codom->pose.pose.position.z
                };
                double ts = codom->header.stamp.toSec();
                trajectory[ts] = mat.matrix();
            }
        }
    }

    std::cout << "Trajectory info :\n";
    std::cout << "\t count      :" << trajectory.size() << "\n";
    std::cout << "\t begin time :" << trajectory.begin()->first << "\n";
    std::cout << "\t end   time :" << trajectory.rbegin()->first << "\n";

    const int max_points = vm["max_points"].as<int>();
    const float max_distance = vm["max_distance"].as<float>();
    const float max_range = vm["max_range"].as<float>();


    Eigen::Affine3d current_odometry{trajectory.begin()->second};

    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    std::vector<resso> resso_data;

    for (const auto &p : patches) {
        std::cout << "processing " << p << std::endl;
        rosbag::Bag bag;
        bag.open(p);  // BagMode is Read by default

        rosbag::View view_data(bag, rosbag::TopicQuery(std::vector<std::string>({"/livox/lidar"})));


        Eigen::Matrix4d last_pose;
        for (rosbag::MessageInstance const m: view_data) {
            livox_ros_driver2::CustomMsgConstPtr cptr = m.instantiate<livox_ros_driver2::CustomMsg>();
            if (cptr) {
                assert(cptr->header.stamp >= last_time);
                last_time = cptr->header.stamp;
                for (const auto &pp : cptr->points) {
                    if (pp.x> max_range || pp.x  < 0.5) continue;
                    double time = cptr->header.stamp.toSec() + static_cast<double>(pp.offset_time) / 1e9;
                    auto interpolated = my_utils::getInterpolatedPose(trajectory, time);
                    if (!interpolated.isZero()) {
                        last_pose = interpolated;
                        auto pose_increment = current_odometry.inverse() * last_pose;
                        pcl::PointXYZI p;
                        p.getVector4fMap() = (pose_increment *
                                              Eigen::Vector4d{pp.x, pp.y, pp.z, 1.0}).cast<float>();
                        p.intensity = pp.reflectivity;
                        pointcloud.push_back(p);
                    }
                }

                float distance = (current_odometry.translation()-last_pose.col(3).head<3>()).norm();
                if (pointcloud.size() >= max_points || distance > max_distance ) {
                    const std::string ply_filename = savePLY(out_dir_name, resso_data.size(), pointcloud);
                    resso_data.push_back(resso{ply_filename, current_odometry.matrix()});

                    pointcloud.clear();
                    current_odometry=Eigen::Affine3d(last_pose);

                }
            }
        }
        bag.close();
    }
    const std::string ply_filename = savePLY(out_dir_name, resso_data.size(), pointcloud);
    resso_data.push_back(resso{ply_filename, current_odometry.matrix()});

    std::ofstream resso_file (out_dir_name+"/data.reg");
    resso_file << resso_data.size() << std::endl;
    for (const auto &p : resso_data){
        resso_file << p.file_name << std::endl;
        resso_file << p.mat << std::endl;
    }
    resso_file.close();
}
