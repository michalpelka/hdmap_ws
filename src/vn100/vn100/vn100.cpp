#include "vn100.h"
#include "vn100_parsing.h"
#include <iostream>
vn100_client::vn100_client(const std::string& portname, int baudrate)
    : portname(portname)
    , baudrate(baudrate)
    , done(false)
{
    this->listner_thread = std::thread(&vn100_client::vn100_client_listener_thread_worker, this);
    this->monitor_thread = std::thread(&vn100_client::vn100_client_monitor_thread_worker, this);

}

void vn100_client::vn100_client_listener_thread_worker(){

    while(!done) {
        BoostSerial serial;
        serial.open(portname, baudrate);
        serial.setTimeout(1000);
        middle_handler_fired_at = 0;
        while (!done) {
            std::string data = serial.readStringUntil(0x0D);
            std::string data_striped = vn100_parsing::getCheckSumData(data);
            auto checksum_inMsg = vn100_parsing::getChecksum(data);
            auto checksum_computed = vn100_parsing::calculateChecksum((unsigned char *) data_striped.c_str(),
                                                                      data_striped.length());
            if (checksum_inMsg == checksum_computed) {

                auto v = vn100_parsing::vnqmr_parse(data_striped);
                if (v.timestamp > last_timestamp) {

                    std::cout << "count_since_turnover " << count_since_turnover << std::endl;
                    this->count_since_turnover = 0;
                }

                const double whole_sec = v.timestamp;
                const double frac_sec = 1.0*count_since_turnover/expected_framerate;
                const double ts_sec = whole_sec+frac_sec;
                if (data_handler){
                    data_handler(ts_sec, v.q,v.mag, v.accel, v.gyro);
                }
//                std::cout << "ts_sec "<< ts_sec << std::endl;
                if (middle_handler_fired_at < whole_sec && frac_sec > 0.5){
                    if(handler_pps_middle){
                        handler_pps_middle(whole_sec, frac_sec);
                    }
                    middle_handler_fired_at = whole_sec;
                }

                last_timestamp = v.timestamp;
                last_timestamp_sent = ts_sec;
                this->count_since_turnover++;
                msgs++;
            }

        }
    }
}

void vn100_client::vn100_client_monitor_thread_worker(){
    while(!done){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        msgs_count_1sec.store(msgs);
        msgs.store(0);
    }
}
