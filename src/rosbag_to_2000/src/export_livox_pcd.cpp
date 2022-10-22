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
#include <boost/program_options.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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

    void susbample_and_save(const pcl::PointCloud<pcl::PointXYZI>& pointcloud, int saved_pc,
                            const std::string& out_dir_name,
                            const Eigen::Affine3d& current_odometry = Eigen::Affine3d(Eigen::Matrix4d::Zero())) {
        char fn[128];
        snprintf(fn, 128, "map_%05d", saved_pc);
        std::string filename = out_dir_name + "/" + fn + ".pcd";
//        std::string filename_floor = out_dir_name + "/" + fn + "_floor.pcd";
//        std::string filename_walls = out_dir_name + "/" + fn + "_walls.pcd";
        std::string filename_mat = out_dir_name + "/" + fn + ".txt";
//        std::string filename_ground = out_dir_name + "/" + fn + "_ground.txt";


        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(pointcloud.makeShared());
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZI> pointcloud_ds;
        sor.filter(pointcloud_ds);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.5);


        seg.setInputCloud(pointcloud_ds.makeShared());
        seg.segment(*inliers, *coefficients);

//        if (coefficients && coefficients->values[2] > 0.5) {
//            pcl::PointCloud<pcl::PointXYZI> pointcloud_floor;
//            pcl::PointCloud<pcl::PointXYZI> pointcloud_wall;
//            pcl::ExtractIndices<pcl::PointXYZI> extract;
//            extract.setInputCloud(pointcloud_ds.makeShared());
//            extract.setIndices(inliers);
//
//            extract.setNegative(false);
//            extract.filter(pointcloud_floor);
//            pcl::io::savePCDFile(filename_floor, pointcloud_floor);
//            extract.setNegative(true);
//            extract.filter(pointcloud_wall);
//            pcl::io::savePCDFile(filename_walls, pointcloud_wall);
//            std::ofstream fmt_ofs (filename_ground);
//            fmt_ofs << coefficients->values[0] <<" "
//                    << coefficients->values[1] <<" "
//                    << coefficients->values[2] <<" "
//                    << coefficients->values[3] << std::endl;
//            fmt_ofs.close();
//
//        }

        pcl::io::savePCDFile(filename, pointcloud_ds);
        if (!current_odometry.matrix().isZero()) {
            my_utils::saveMat(filename_mat, current_odometry.matrix());
        }
}
int main(int ac, char** av)
{
    po::variables_map vm;
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
                ("help", "produce help message")
                ("dir", po::value<std::string>(), "directory to work")
                ("save_global", po::value<bool>()->default_value(0), "save points in local co")
                ("max_points", po::value<int>()->default_value(1e6), "max no points in one pcd")
                ("max_range", po::value<float>()->default_value(25), "max range");


        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);


    }
    catch(const std::exception &e){
        std::cerr << e.what() << std::endl;
        return 1;
    }
    const std::string fn{vm["dir"].as<std::string>()};
    const float max_range = vm["max_range"].as<float>()  ;
    const std::string out_dir_name {fn+"/data_pcd_map"};
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

    bool save_global = vm["save_global"].as<bool>();
    std::cout << "save_global " << save_global << std::endl;
    const int max_points = vm["max_points"].as<int>();
    if (save_global) {
        int saved_pc = 0;
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        for (const auto &p : patches) {
            std::cout << "processing " << p << std::endl;
            rosbag::Bag bag;
            bag.open(p);  // BagMode is Read by default

            rosbag::View view_data(bag, rosbag::TopicQuery(std::vector<std::string>({"/livox/lidar"})));
            for (rosbag::MessageInstance const m: view_data) {
                livox_ros_driver::CustomMsgConstPtr cptr = m.instantiate<livox_ros_driver::CustomMsg>();
                if (cptr) {
                    assert(cptr->header.stamp >= last_time);
                    last_time = cptr->header.stamp;
                    for (const auto &pp : cptr->points) {
                        if (pp.x> max_range || pp.x  < 1.0) continue;
                        double time = cptr->header.stamp.toSec() + static_cast<double>(pp.offset_time) / 1e9;
                        auto pose = my_utils::getInterpolatedPose(trajectory, time);
                        pcl::PointXYZI p;
                        p.getVector4fMap() = (pose * Eigen::Vector4d{pp.x, pp.y, pp.z, 1.0}).cast<float>();
                        p.intensity = pp.reflectivity;
                        pointcloud.push_back(p);
                    }

                    if (pointcloud.size() >= max_points) {

                        susbample_and_save(pointcloud, saved_pc, out_dir_name);

                        pointcloud.clear();
                        saved_pc++;

                    }
                }
            }
            bag.close();
        }
        susbample_and_save(pointcloud, saved_pc, out_dir_name);
    }
    else
    {
        Eigen::Affine3d current_odometry{trajectory.begin()->second};
        int saved_pc = 0;

        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        for (const auto &p : patches) {
            std::cout << "processing " << p << std::endl;
            rosbag::Bag bag;
            bag.open(p);  // BagMode is Read by default

            rosbag::View view_data(bag, rosbag::TopicQuery(std::vector<std::string>({"/livox/lidar"})));


            Eigen::Matrix4d last_pose;
            for (rosbag::MessageInstance const m: view_data) {
                livox_ros_driver::CustomMsgConstPtr cptr = m.instantiate<livox_ros_driver::CustomMsg>();
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

                    if (pointcloud.size() >= max_points) {

                        susbample_and_save(pointcloud, saved_pc, out_dir_name,current_odometry);
                        pointcloud.clear();
                        saved_pc++;
                        current_odometry=Eigen::Affine3d(last_pose);

                    }
                }
            }
            bag.close();
        }
        susbample_and_save(pointcloud, saved_pc, out_dir_name,current_odometry);
    }
}
