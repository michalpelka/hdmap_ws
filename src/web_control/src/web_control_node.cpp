#include "health_server.h"
#include "subprocess.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


#include <thread>
#include <mutex>
#include <chrono>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <livox_ros_driver2/CustomMsg.h>

using LivoxCustomMsg= livox_ros_driver2::CustomMsg;
const std::string kEnvRepo = "HD_Repo";

const std::vector<std::string> topics_to_record{
        "/livox/imu",
        "/livox/lidar",
        "/time",
};
std::string GetEnv( const std::string & var ) {
    const char * val = std::getenv( var.c_str() );
    if ( val == nullptr ) { // invalid to assign nullptr to std::string
        return "";
    }
    else {
        return val;
    }
}

double clamp(double v)
{
    const double t = v < 0 ? 0 : v;
    return t > 1.0 ? 1.0 : t;
}

namespace ds{
    std::mutex global_lock;
    std::mutex img_lock;
    std::string current_record_dir;
    std::shared_ptr<subprocess::Popen> bag_process{nullptr};
    std::shared_ptr<subprocess::Popen> ubx_process{nullptr};

    cv::Mat img;
    cv::Mat img_thumb;
    cv::Mat img_livox;
    unsigned int counter_images = 0;
    unsigned int counter_livox = 0;
    unsigned int counter_ublox = 0;
    std::string status_ublox;
    std::string accuracy;

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        std::lock_guard<std::mutex> lck(ds::img_lock);
        const auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::resize(img, ds::img_thumb,cv::Size(), 0.1,0.1);
        ds::img = img.clone();
        std::lock_guard<std::mutex> lck2(ds::global_lock);
        ds::counter_images++;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void livoxCallback(const sensor_msgs::Imu& msg)
{
    std::lock_guard<std::mutex> lck(ds::global_lock);
    ds::counter_livox++;
}
void livoxCallback2(const LivoxCustomMsg& msg)
{
    int k  = 512;
    cv::Mat m = cv::Mat(k,k,CV_8UC3, cv:: Scalar(10, 100, 150));

    for (auto &p : msg.points){
        float xx = 15.0*p.x;
        float yy = 15*p.y;
        int xxx = std::round(xx+k/2);
        int yyy = std::round(yy+k/2);

        if (xxx > 0 && xxx <k && yyy >0&& yyy<k   ){
            auto t =clamp(p.x/20);
            double red   = clamp(1.5 - std::abs(2.0 * t - 1.0));
            double green = clamp(1.5 - std::abs(2.0 * t));
            double blue  = clamp(1.5 - std::abs(2.0 * t + 1.0));
            m.at<cv::Vec3b>(xxx,yyy) = cv::Vec3b(255*red,255*green,255*blue);
        }

    }
    std::lock_guard<std::mutex> lck2(ds::img_lock);
    ds::img_livox = m.clone();

}
void ubloxFixCallback(const sensor_msgs::NavSatFix &gps)
{
    std::lock_guard<std::mutex> lck(ds::global_lock);
    ds::counter_ublox++;
}

void diagnosticCallback(const diagnostic_msgs::DiagnosticArray &array)
{
    for (const auto &entry : array.status)
    {
        if (entry.hardware_id == "ublox"){
            std::lock_guard<std::mutex> lck2(ds::global_lock);
            ds::status_ublox = entry.message;
            ds::accuracy="";
            for (auto &v : entry.values){
                if (v.key.find("Accuracy") != std::string::npos){
                    ds::accuracy+= v.key+" : "+v.value+"\n";
                }
            }
        }
    }
}

std::vector<uint8_t> produceImgThumb() {
    std::lock_guard<std::mutex> lck(ds::img_lock);
    std::vector<uint8_t> buf;
    if (ds::img_thumb.empty()){
        return buf;
    }
    cv::imencode(".JPG", ds::img_thumb, buf );
    return buf;
}

std::vector<uint8_t> produceImg() {
    std::lock_guard<std::mutex> lck(ds::img_lock);
    std::vector<uint8_t> buf;
    if (ds::img_livox.empty()){
        return buf;
    }
    cv::imencode(".JPG", ds::img_livox, buf );
    return buf;
}

std::vector<uint8_t> produceImgLivox() {
    std::lock_guard<std::mutex> lck(ds::img_lock);
    std::vector<uint8_t> buf;
    if (ds::img_livox.empty()){
        return buf;
    }
    cv::imencode(".JPG", ds::img_livox, buf );
    return buf;
}


std::string produceReport() {
    std::lock_guard<std::mutex> lck(ds::global_lock);
    boost::property_tree::ptree pt;
    static int number=0;
    pt.put("number", number++);

    // msgs
    pt.put("status.image.count",ds::counter_images);
    pt.put("status.livox.count",ds::counter_livox);
    pt.put("status.ublox.count",ds::counter_ublox);
    pt.put("status.ublox.accuracy",ds::accuracy);
    pt.put("status.ublox.status",ds::status_ublox);


    // bag
    pt.put("status.bag.directory",ds::current_record_dir);
    int pid = -1;
    int ret = 99;
    if (ds::bag_process){
        pid = ds::bag_process->pid();
        ret = ds::bag_process->retcode();
    }
    pt.put("status.bag.pid", pid);
    pt.put("status.bag.ret", ret);

    namespace fs = boost::filesystem;
    std::stringstream file_list;
    if (fs::exists(ds::current_record_dir)) {
        std::multimap<std::time_t, fs::path> dir_result_set;
        for (const fs::directory_entry &f: fs::recursive_directory_iterator(ds::current_record_dir)) {
            if (fs::is_regular_file(f.path())) {
                const auto mod_time = fs::last_write_time(f.path());
                dir_result_set.insert(std::pair<std::time_t, fs::path>(mod_time, f.path()));
            }
        }
        for (const auto &file_sorted: dir_result_set) {
            const auto &f = file_sorted.second;
            file_list << f.filename() << "   [ " << std::fixed << double(fs::file_size(f)) / (1024 * 1024 * 1024)
                      << " ]\n";
        }
    }
    pt.put("status.bag.file_list", file_list.str());

    // disk space
    boost::filesystem::space_info si = boost::filesystem::space("/");
    pt.put("status.system.disk", float(si.available) / (1024*1024*1024));

    std::stringstream oss;
    boost::property_tree::write_json(oss, pt);
    return oss.str();
}

std::string start_recording(const std::string& tt){
    namespace fs = boost::filesystem;
    if(ds::bag_process) return "nok";
    std::lock_guard<std::mutex> lck(ds::global_lock);

    using sysclock_t = std::chrono::system_clock;
    std::time_t now = sysclock_t::to_time_t(sysclock_t::now());
    char buf[256] = { 0 };
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    const auto record_dir = fs::path(GetEnv(kEnvRepo))/(std::string(buf));
    fs::create_directory(record_dir);
    ds::current_record_dir = record_dir.string();

    std::cout <<"logging to " << ds::current_record_dir << std::endl;
    std::vector<std::string> params_to_bag_process{"rosbag","record",
                                                   "--split","--duration=2m","-o",(ds::current_record_dir+"/log")};
    std::copy(std::begin(topics_to_record),std::end(topics_to_record), std::back_inserter(params_to_bag_process));

    ds::bag_process = std::shared_ptr<subprocess::Popen>(new subprocess::Popen(params_to_bag_process));


    return "ok";
}

std::string stop_recording(const std::string& tt){
    std::lock_guard<std::mutex> lck(ds::global_lock);
    if (ds::bag_process){
        ds::bag_process->close_input();
        ds::bag_process->kill(2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ds::bag_process->kill(9);
        ds::bag_process = nullptr;
    }
    if (ds::ubx_process){
        ds::ubx_process->close_input();
        ds::ubx_process->kill(2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ds::ubx_process->kill(9);
        ds::ubx_process = nullptr;
    }
    return "ok";
}

int main(int argc, char**argv){
    health_server::setStatusHandler(produceReport);
    std::thread http_thread1(health_server::server_worker);
    health_server::setTriggerHandler(start_recording, "start_bag");
    health_server::setTriggerHandler(stop_recording, "stop_bag");

    health_server::setDataHandler(produceImgThumb, "img_thumb");
    health_server::setDataHandler(produceImg, "img_full");
    health_server::setDataHandler(produceImgLivox, "img_livox");


    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    ros::Subscriber sub1 = nh.subscribe("/livox/imu",1, livoxCallback);
    ros::Subscriber sub2 = nh.subscribe("/ublox/fix",1, ubloxFixCallback);
    ros::Subscriber sub22 = nh.subscribe("/fix",1, ubloxFixCallback);

    ros::Subscriber sub3 = nh.subscribe("/diagnostics",1, diagnosticCallback);
    ros::Subscriber sub4 = nh.subscribe("/livox/lidar",1, livoxCallback2);
    ros::spin();

    return 0;
}
