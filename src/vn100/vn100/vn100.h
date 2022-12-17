#pragma once
#include "thread"
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <BoostSerial.h>


class vn100_client {
public:
    vn100_client(const std::string& portname, int baudrate=230400);
    std::string getReport() const;
    int getRate() {
        return msgs_count_1sec;
    }

    void setDone(){
        done.store(true);
    }

    bool getDone(){
        return done;
    }
    void setHandler_pps_change(const std::function<void(double, double)> &_handler_pps_change) {
        handler_pps_middle = _handler_pps_change;
    }

    void setHandler_data(const std::function<void(double,std::array<double,4>,
            std::array<double,3>,std::array<double,3>,std::array<double,3>)> &_data_handler) {
        data_handler = _data_handler;
    }

private:
    void vn100_client_listener_thread_worker();
    void vn100_client_monitor_thread_worker();

    std::string portname;
    int baudrate;
    std::string data;
    std::thread listner_thread;
    std::thread monitor_thread;

    mutable std::mutex mtx;
    std::atomic<bool> done;
    std::atomic<int> msgs;
    std::atomic<int> msgs_count_1sec;

    int count_since_turnover = 0;
    int last_timestamp = 0;
    const float expected_framerate=101;
    double middle_handler_fired_at=0;
    double last_timestamp_sent = 0;
    std::function<void(double, double)> handler_pps_middle;
    std::function<void(double,std::array<double,4>,std::array<double,3>,
            std::array<double,3>,std::array<double,3>)> data_handler;
};
