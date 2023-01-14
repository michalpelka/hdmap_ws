#include <boost/thread.hpp>
#include <ros/ros.h>
#include "vn100/vn100.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Dense>
class imu_driver{
private:
    ros::NodeHandle n;

    ros::Publisher pub_imu_wall;
    ros::Publisher pub_imu_hardware;
    std::shared_ptr<vn100_client> vn100_client_1;
    int imu_rate;

public:
    void timerCallback(const ros::TimerEvent&)
    {
        if (vn100_client_1){
            ROS_INFO("VN100 rate %d", vn100_client_1->getRate());
        }
    }

    imu_driver():n()
    {
        boost::asio::io_service io_service;

        vn100_client_1 = std::make_shared<vn100_client>("/dev/ttyUSB0", 230400);
        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

        pub_imu_wall = n.advertise<sensor_msgs::Imu>("/vn100/imu_wall", 5);
        pub_imu_hardware  = n.advertise<sensor_msgs::Imu>("/vn100/imu", 5);


        const auto imu_data_handler = [&](double ts,
                std::array<float,4> q, std::array<float,3> acc,std::array<float,3>gyro){
            sensor_msgs::Imu imu_msg;

            Eigen::Matrix3f m;
            m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitX());

            imu_msg.orientation.x = q[0];
            imu_msg.orientation.y = q[1];
            imu_msg.orientation.z = q[2];
            imu_msg.orientation.w = q[3];

            Eigen::Vector3f gyro_raw(gyro[0],gyro[1],gyro[2]);
            Eigen::Vector3f gyro_rawt = m* gyro_raw;
            imu_msg.angular_velocity.x = gyro_rawt.x();
            imu_msg.angular_velocity.y = gyro_rawt.y();
            imu_msg.angular_velocity.z = gyro_rawt.z();

            Eigen::Vector3f acc_raw(acc[0],acc[1],acc[2]);
            Eigen::Vector3f acc_rawt = m* acc_raw;
            imu_msg.linear_acceleration.x = acc_rawt.x();
            imu_msg.linear_acceleration.y = acc_rawt.y();
            imu_msg.linear_acceleration.z = acc_rawt.z();

            imu_msg.linear_acceleration_covariance[0] = 0.0001;
            imu_msg.linear_acceleration_covariance[4] = 0.0001;
            imu_msg.linear_acceleration_covariance[8] = 0.0001;

            imu_msg.angular_velocity_covariance[0] = 0.001;
            imu_msg.angular_velocity_covariance[4] = 0.001;
            imu_msg.angular_velocity_covariance[8] = 0.001;

            imu_msg.orientation_covariance[0] = 0.1;
            imu_msg.orientation_covariance[4] = 0.1;
            imu_msg.orientation_covariance[8] = 0.1;

            imu_msg.header.frame_id="livox";
            imu_msg.header.stamp = ros::Time::now();
            pub_imu_wall.publish(imu_msg);

            imu_msg.header.stamp.fromSec(ts);
            pub_imu_hardware.publish(imu_msg);
        };

        vn100_client_1->setHandler_data(imu_data_handler);
        const auto timer = n.createTimer(ros::Duration(1), &imu_driver::timerCallback, this, false);

        ros::spin();
    }


};

int main(int argc, char** argv) {
    // Initialize ROS

    ros::init(argc, argv, "native_sync");
    imu_driver();
}
