#pragma once
#include <string>
#include <array>

namespace vn100_parsing {
    struct vnqmr_data {
        std::array<double, 4> q;
        std::array<double, 3> mag;
        std::array<double, 3> accel;
        std::array<double, 3> gyro;
        int timestamp;
    };

    unsigned char calculateChecksum(unsigned char data[], unsigned int length);

    std::string getCheckSumData(const std::string &data);

    unsigned getChecksum(const std::string &data);

    unsigned getChecksum(const std::string& data );

    vnqmr_data vnqmr_parse(std::string data );

}