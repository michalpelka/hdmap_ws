#include "ros/ros.h"
#include "livox_ros_driver/CustomPoint.h"
#include "livox_ros_driver/CustomMsg.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "memory"
#include <deque>

double getFrac(double d){
   double frac =  d - std::floor(d);
   return frac;
}

class LivoxDataSource{
public:
    LivoxDataSource(const std::string& livoxName, const std::string& livoxTopicName, unsigned int lineNumber,  ros::NodeHandle& n ):
            m_livoxName(livoxName),
            m_startNumberOfLine(lineNumber)
    {
        m_inputMessages = std::make_shared<ros::Subscriber>(n.subscribe(livoxTopicName, 1, &LivoxDataSource::messageFromLivox, this));
    }
private:
    void messageFromLivox(const livox_ros_driver::CustomMsgConstPtr data){
        m_pointCount+=data->point_num;
        livox_ros_driver::CustomMsg resultMsg;
        resultMsg.point_num = data->point_num;
        resultMsg.header = data->header;
        resultMsg.lidar_id = 0;

        resultMsg.rsvd = data->rsvd;
        resultMsg.points.resize(data->point_num);
        const double livoxTimeStamp = 1.0*data->timebase / 1e9;
        if (getFrac(m_oldLivoxTime) > 0.95) {
            if (getFrac(livoxTimeStamp) < .1)
            {
                // trunover on livox
                ROS_INFO("PPS on %s %f -> %f", m_livoxName.c_str(), m_currentWhole, m_currentWhole+1);
                m_currentWhole+=1.0;
            }
        }
        m_oldLivoxTime = livoxTimeStamp;
        const double resolved_tts = m_currentWhole+getFrac(livoxTimeStamp);
        resultMsg.header.stamp.fromSec(resolved_tts);
        resultMsg.timebase = resultMsg.header.stamp.toNSec();
        for (int i=0; i < data->point_num; i++){

            const auto &ip = data->points[i];
            Eigen::Vector3f ipf(ip.x, ip.y, ip.z);
            Eigen::Vector3f opf = m_calibration * ipf;
            livox_ros_driver::CustomPoint pf;
            pf.offset_time = ip.offset_time;
            pf.tag = ip.tag;
            pf.reflectivity = ip.reflectivity;
            pf.x = opf.x();
            pf.y = opf.y();
            pf.z = opf.z();
            pf.line= ip.line + this->m_startNumberOfLine;
            resultMsg.points[i] = pf;
        }
        m_buffer.push_back(resultMsg);
    }
public:
    void setCurrentTimestamp(double timestamp){
        double whole = std::floor(timestamp);
        std::cout <<"Synchronization on " << m_livoxName <<" : "<< timestamp <<" (" << m_currentWhole <<")"<< std::endl;
        if (whole != m_currentWhole){
            std::cout <<"Error on synchronization on " << m_livoxName  <<" : "<< timestamp <<" (" << m_currentWhole <<")" << std::endl;
        }
        m_currentWhole = whole;
    }

    std::string m_livoxName;
    std::shared_ptr<ros::Subscriber> m_inputMessages;
    double m_wholeSecTimeStamp{0};
    unsigned int m_startNumberOfLine{1};
    Eigen::Affine3f m_calibration;
    std::deque<livox_ros_driver::CustomMsg> m_buffer;
    int m_pointCount =0;
    double m_currentWhole = 0;
    double m_oldLivoxTime = 0;
    int m_lidarId=0;

    void SetCalibration (const Eigen::Affine3f & f){
        m_calibration = f;
    }
    std::deque<livox_ros_driver::CustomMsg>& getBuffer(){
        return m_buffer;
    }
};

class LivoxDataSourceImu {
private:
    std::unique_ptr<ros::Publisher> publisher;
public:
    LivoxDataSourceImu(const std::string &livoxName, const std::string &livoxTopicNameImu,
                       const std::string &livoxRepublish, ros::NodeHandle &n) : m_livoxName(livoxName)
    {
        m_inputMessages = std::make_shared<ros::Subscriber>(
        n.subscribe(livoxTopicNameImu, 1, &LivoxDataSourceImu::messageFromLivoxImu, this));
        publisher = std::make_unique<ros::Publisher>(n.advertise<sensor_msgs::Imu>(livoxRepublish, 10));
    }

private:
    void messageFromLivoxImu(const sensor_msgs::ImuConstPtr data) {
        sensor_msgs::Imu resultMsg = *data;
        const double livoxTimeStamp = 1.0 * data->header.stamp.toSec();
        if (getFrac(m_oldLivoxTime) > 0.95) {
            if (getFrac(livoxTimeStamp) < .1) {
                // trunover on livox
                ROS_INFO("PPS on %s %f -> %f", m_livoxName.c_str(), m_currentWhole, m_currentWhole + 1);
                m_currentWhole += 1.0;
            }
        }
        m_oldLivoxTime = livoxTimeStamp;
        const double resolved_tts = m_currentWhole + getFrac(livoxTimeStamp);
        resultMsg.header.stamp.fromSec(resolved_tts);
        publisher->publish(resultMsg);
    }

public:
    void setCurrentTimestamp(double timestamp) {
        double whole = std::floor(timestamp);
        std::cout << "Synchronization on " << m_livoxName << " : " << timestamp << " (" << m_currentWhole << ")"
                  << std::endl;
        if (whole != m_currentWhole) {
            std::cout << "Error on synchronization on " << m_livoxName << " : " << timestamp << " (" << m_currentWhole
                      << ")" << std::endl;
        }
        m_currentWhole = whole;
    }

    std::string m_livoxName;
    std::shared_ptr<ros::Subscriber> m_inputMessages;
    double m_wholeSecTimeStamp{0};

    double m_currentWhole = 0;
    double m_oldLivoxTime = 0;
};



std::vector<std::shared_ptr<LivoxDataSource>> livoxDataSources;
std::shared_ptr<LivoxDataSourceImu> livoxDataSourcesImu;

std::unique_ptr<ros::Publisher> livox_result;
std::unique_ptr<ros::Publisher> imu_synch;

void timerCallback(const ros::TimerEvent&){
    livox_ros_driver::CustomMsg msg;
    bool init = false;
    for (auto & src : livoxDataSources){
        for (auto &curr_msg : src->getBuffer()){
            if (!init){
                msg = curr_msg;
                init = true;
            }
            else{
                for (auto point : curr_msg.points){
                    double ts = curr_msg.header.stamp.toSec() + point.offset_time;
                    point.offset_time = ts - msg.header.stamp.toSec();
                    msg.points.push_back(point);
                }
            }
        }
        src->getBuffer().clear();
    }
    msg.point_num = msg.points.size();
    livox_result->publish(msg);
}

double imu_timer = 0;
double imu_old_ts = 0;
bool imu_to_send = false;

void imu_callback(const sensor_msgs::ImuConstPtr &imu){
    double imu_curr_ts = imu->header.stamp.toSec();
    if (getFrac(imu_old_ts) > 0.5) {
        if (getFrac(imu_curr_ts) < .4)
        {
            // trunover on livox
            ROS_INFO("PSS on imu");
            imu_timer+=1.0;
            imu_to_send = true;
        }
    }
    imu_old_ts = imu_curr_ts;
    if (imu_to_send && getFrac(imu_curr_ts)>0.5) {
        for (auto &src : livoxDataSources) {
            src->setCurrentTimestamp(imu_timer + getFrac(imu_curr_ts));
        }
        livoxDataSourcesImu->setCurrentTimestamp(imu_timer + getFrac(imu_curr_ts));
        imu_to_send = false;
    }
    sensor_msgs::Imu new_msg = *imu;
    new_msg.linear_acceleration.x *=0.1f;
    new_msg.linear_acceleration.y *=0.1f;
    new_msg.linear_acceleration.z *=0.1f;

    new_msg.header.stamp.fromSec(imu_timer + getFrac(imu_curr_ts));
    if(imu_synch){
        imu_synch->publish(new_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_mux");
    ros::NodeHandle n;
    livox_result = std::make_unique<ros::Publisher>(n.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 1000));
    imu_synch = std::make_unique<ros::Publisher>(n.advertise<sensor_msgs::Imu>("/livox/imu", 1000));
    Eigen::Affine3f f1{Eigen::Matrix4f::Identity()};
    Eigen::Affine3f f2{Eigen::Matrix4f::Identity()};
    Eigen::Affine3f f3{Eigen::Matrix4f::Identity()};

    f1.translation() = Eigen::Vector3f{0,0,0};
    f2.translation() = Eigen::Vector3f{0,0,0};

    f2.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));

//    auto livox_imu = n.subscribe("/avia/livox/imu",10,imu_callback);
    auto livox_imu = n.subscribe("/vn100/imu",10,imu_callback);


    std::shared_ptr<LivoxDataSource> src1 = std::make_shared<LivoxDataSource>("front", "/avia/livox/lidar", 0, n);
    src1->SetCalibration(f1);
    livoxDataSources.push_back(src1);

    livoxDataSourcesImu = std::make_shared<LivoxDataSourceImu>("frontImu", "/avia/livox/imu", "/livox/imu_avia", n);


    std::shared_ptr<LivoxDataSource> src2 = std::make_shared<LivoxDataSource>("side", "/mid70/livox/lidar", 6, n);
    src2->SetCalibration(f2);
    livoxDataSources.push_back(src2);


    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

    timer.start();
    ros::spin();
}