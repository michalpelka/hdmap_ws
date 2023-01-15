#include "vn100.h"
#include "vn100_parsing.h"
#include <iostream>
#include "Hexdump.hpp"
vn100_client::vn100_client(const std::string& portname, int baudrate)
    : portname(portname)
    , baudrate(baudrate)
    , done(false)
{
    this->listner_thread = std::thread(&vn100_client::vn100_client_listener_thread_worker, this);
    this->monitor_thread = std::thread(&vn100_client::vn100_client_monitor_thread_worker, this);

}

unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
    unsigned int i;
    unsigned short crc = 0;
    for (i = 0; i < length; i++) {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
}

bool checkCRC(const std::vector<uint8_t>& data) {
    if (data.size() < 9)
    {
        return false;
    }
    const auto ptr_crc = data.end()-2;
    const auto ptr_payload_start = data.begin() + 5;

    uint16_t crc_read = (*ptr_crc) << 8 | *(ptr_crc + 1);

    uint16_t crc_computed = calculateCRC((unsigned char*)data.data() + 1, data.size()-3);
    return crc_read == crc_computed;
}

std::vector<int> find_headers(const std::vector<uint8_t>& header_expected1, const std::vector<uint8_t>& buffer ){
    std::vector<int> headers;
    const int buffer_end = buffer.size() - header_expected1.size();
    for (int i = 0; i < buffer_end; i++){
        bool is_e = std::equal(header_expected1.begin(), header_expected1.end(), buffer.begin()+i);
        if (is_e){
            headers.push_back(i);
        }
    }
    return headers;
}

void vn100_client::vn100_client_listener_thread_worker(){


    while(!done) {
        BoostSerial serial;
        serial.open(portname, baudrate);

        serial.setTimeout(10);
        middle_handler_fired_at = 0;

        std::vector<uint8_t> buffer;
        serial.flush();
        while (!done) {
            std::vector<uint8_t> chunk = serial.readBuffer();
            if(chunk.size()){
                std::move(chunk.begin(), chunk.end(), std::back_inserter(buffer));
            }
            std::vector<int> header_starts = find_headers(header_expected1, buffer);

            if (header_starts.size()>=2){
                std::vector<uint8_t> data;
                copy(buffer.begin()+header_starts[0], buffer.begin()+header_starts[1], back_inserter(data));
                if(checkCRC(data)){
                    VN100BINARY::VN100_group1 parsed;
                    VN100BINARY::parse(data,parsed);
                    const double timestamp = static_cast<double>(parsed.TimeStartup) / 1e9;
                    if (data_handler){
//                        std::array<float,3> acc{parsed.Imu[0],parsed.Imu[1],parsed.Imu[2]};
//                        std::array<float,3> gyro{parsed.Imu[3],parsed.Imu[4],parsed.Imu[5]};
                        std::array<float,3> acc{parsed.Accel};
                        std::array<float,3> gyro{parsed.AngularRate};

                        data_handler(timestamp, parsed.Quaternion, acc,gyro);
                    }
                    const double whole_sec = std::floor(timestamp);
                    const double frac_sec = timestamp - whole_sec;
                    if (middle_handler_fired_at < whole_sec && frac_sec > 0.5){
                        if(handler_pps_middle){
                            handler_pps_middle(whole_sec, frac_sec);
                        }
                        middle_handler_fired_at = whole_sec;
                    }
                    msgs++;
                }
                else{
                    std::cout << "data with wrong CRC : " <<Hexdump(data.data(), data.size())<<std::endl;
                }
                buffer.erase(buffer.begin(), buffer.begin()+header_starts.back());
            }
            if (buffer.size() > 32*1024){
                std::cerr << "Buffer without any headers! smth went horribly wrong" << std::endl;
                std::cerr << "Maybe config of imu is wrong or " << std::endl;

                std::cerr <<Hexdump(buffer.data(), buffer.size())<<std::endl;
                buffer.clear();
            }
//
//            std::vector<uint8_t>header = serial.readBytesUntil(header_expected, 128);
//
//            if (header==header_expected){
//
//                std::vector<uint8_t>data = serial.readBuffer();
//                std::cout << "data " << data.size() << std::endl;
//
//                msgs++;
//            }


//            std::cout << "data " << header.size() << std::endl;



//            std::string data_striped = vn100_parsing::getCheckSumData(data);
//
//            auto checksum_inMsg = vn100_parsing::getChecksum(data);
//            auto checksum_computed = vn100_parsing::calculateChecksum((unsigned char *) data_striped.c_str(),
//                                                                      data_striped.length());
//            if (checksum_inMsg == checksum_computed) {
//
//                auto v = vn100_parsing::vnqmr_parse(data_striped);
//                if (v.timestamp > last_timestamp) {
//
//                    std::cout << "count_since_turnover " << count_since_turnover << std::endl;
//                    this->count_since_turnover = 0;
//                }
//
//                const double whole_sec = v.timestamp;
//                const double frac_sec = 1.0*count_since_turnover/expected_framerate;
//                const double ts_sec = whole_sec+frac_sec;
//                if (data_handler){
//                    data_handler(ts_sec, v.q,v.mag, v.accel, v.gyro);
//                }
////                std::cout << "ts_sec "<< ts_sec << std::endl;
//                if (middle_handler_fired_at < whole_sec && frac_sec > 0.5){
//                    if(handler_pps_middle){
//                        handler_pps_middle(whole_sec, frac_sec);
//                    }
//                    middle_handler_fired_at = whole_sec;
//                }
//
//                last_timestamp = v.timestamp;
//                last_timestamp_sent = ts_sec;
//                this->count_since_turnover++;
//                msgs++;
//            }
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
