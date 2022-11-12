#pragma once
#include <Eigen/Dense>
#include <glob.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <boost/filesystem.hpp>


namespace my_utils{

    Eigen::Matrix4d loadMat(const std::string& fn){
        Eigen::Matrix4d m;
        std::ifstream ifss(fn);
        for (int i =0; i < 16; i++) {
            ifss >> m.data()[i];
        }
        ifss.close();
        std::cout << m.transpose() << std::endl;
        return m.transpose();;
    }
    void saveMat(const std::string& fn, const Eigen::Matrix4d& mat){
        Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
        std::ofstream fmt_ofs (fn);
        fmt_ofs << mat.format(HeavyFmt);
        fmt_ofs.close();
    }
    Eigen::Matrix4d orthogonize(const Eigen::Matrix4d & p )
    {
        Eigen::Matrix4d ret = p;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(ret.block<3,3>(0,0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        double d = (svd.matrixU() * svd.matrixV().transpose()).determinant();
        Eigen::Matrix3d diag = Eigen::Matrix3d::Identity() * d;
        ret.block<3,3>(0,0) = svd.matrixU() * diag * svd.matrixV().transpose();
        return ret;
    }

    std::vector<std::string> findFiles(const std::string & fn, const std::string& extension){
        using namespace boost::filesystem;
        recursive_directory_iterator dir_it(fn);
        std::vector<std::string> patches;
        while (dir_it!=recursive_directory_iterator{})
        {
            if (dir_it->path().extension() == extension){
                patches.push_back(dir_it->path().string());
            }
            dir_it++;
        }
        std::sort(patches.begin(),patches.end());
        return patches;
    }

    Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time)
    {
        Eigen::Matrix4d ret(Eigen::Matrix4d::Zero());
        auto it_lower = trajectory.lower_bound(query_time);
        auto it_next = it_lower;
        if (it_lower == trajectory.begin()){
            return ret;
        }
        if (it_lower->first > query_time) {
            it_lower = std::prev(it_lower);
        }
        if (it_lower == trajectory.begin()){
            return ret;
        }
        if (it_lower == trajectory.end()){
            return ret;
        }

        double t1 = it_lower->first;
        double t2 = it_next->first;
        double difft1 = t1- query_time;
        double difft2 = t2- query_time;
        if (t1 == t2 && std::fabs(difft1)< 0.1){
            ret = Eigen::Matrix4d::Identity();
            ret.col(3).head<3>() = it_next->second.col(3).head<3>();
            ret.topLeftCorner(3,3) = it_lower->second.topLeftCorner(3,3);
            return ret;
        }
        if (std::fabs(difft1)< 0.15 && std::fabs(difft2)< 0.15 )
        {
            assert(t2>t1);
            assert(query_time>t1);
            assert(query_time<t2);
            ret = Eigen::Matrix4d::Identity();
            double res = (query_time-t1)/(t2-t1);
            Eigen::Vector3d diff = it_next->second.col(3).head<3>() - it_lower->second.col(3).head<3>();
            ret.col(3).head<3>() = it_next->second.col(3).head<3>() + diff*res;
            Eigen::Matrix3d r1 = it_lower->second.topLeftCorner(3,3).matrix();
            Eigen::Matrix3d r2 = it_next->second.topLeftCorner(3,3).matrix();
            Eigen::Quaterniond q1(r1);
            Eigen::Quaterniond q2(r2);
            Eigen::Quaterniond qt = q1.slerp(res, q2);
            ret.topLeftCorner(3,3) =  qt.toRotationMatrix();
            return ret;
        }
        std::cout << "Problem with : " <<  difft1 << " " << difft2 << "  q : " << query_time<< " t1 :"<<t1 <<" t2: "<<t2 << std::endl;
        return ret;
    }
    std::vector<bool> downsample (const std::vector<Eigen::Vector3f>& pointcloud, const float voxel_size){
        auto hash = [=](const Eigen::Vector3f& p){
            const int i = (1000*voxel_size)+std::round(p.x()/voxel_size);
            const int j = (1000*voxel_size)+std::round(p.y()/voxel_size);
            const int k = (1000*voxel_size)+std::round(p.z()/voxel_size);
            return uint64_t (1000*i + 1000*1000*j +k);
        };
        std::unordered_map<uint64_t , Eigen::Vector3f> map ;
        std::vector<bool> ret;
        ret.resize(pointcloud.size());
        unsigned int points_to_keep{};
        for (int i =0; i < pointcloud.size(); i++){
            const auto & p  = pointcloud[i];
            auto hashed_point = hash(p);
            const auto insert_result = map.insert(std::make_pair(hashed_point, p));
            ret[i] = insert_result.second;
            if ( ret[i] ){
                points_to_keep++;
            }
        }
        std::cout << "Filtering pointclouds, keep " << 100.0f*points_to_keep/pointcloud.size() << std::endl;
        return ret;
    }

}