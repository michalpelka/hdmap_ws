#include "vn100_parsing.h"
#include "algorithm"
#include "sstream"

unsigned char vn100_parsing::calculateChecksum(unsigned char data[], unsigned int length)
{
    unsigned int i;
    unsigned char cksum = 0;
    for(i=0; i<length; i++){
        cksum ^= data[i];
    }
    return cksum;
}

std::string vn100_parsing::getCheckSumData(const std::string& data )
{
    const auto location_dollar = data.find('$');
    const auto location_asterisk = data.rfind('*');
    if (location_dollar <location_asterisk ) {
        const auto len = location_asterisk - location_dollar;
        return data.substr(location_dollar+1,len-1);
    }
    return std::string ();
}

unsigned vn100_parsing::getChecksum(const std::string& data ){
    const auto location_asterisk = data.rfind('*');
    if(location_asterisk == std::string::npos)
    {
        return -1;
    }
    std::string checksum_str = data.substr(location_asterisk+1);
    try{
        return std::stoi(checksum_str, 0,16);
    }
    catch(...){
        return -1;
    }
}

vn100_parsing::vnqmr_data vn100_parsing::vnqmr_parse(std::string data ){
    vnqmr_data q;
    std::replace(data.begin(),data.end(),',', ' ');
    std::stringstream oss(data);
    std::string str;
    oss >> str;
    std::string timestamp;
    if (str == "VNQMR"){
        oss>>q.q[0];
        oss>>q.q[1];
        oss>>q.q[2];
        oss>>q.q[3];
        oss>>q.mag[0];
        oss>>q.mag[1];
        oss>>q.mag[2];
        oss>>q.accel[0];
        oss>>q.accel[1];
        oss>>q.accel[2];
        oss>>q.gyro[0];
        oss>>q.gyro[1];
        oss>>q.gyro[2];
        oss>> timestamp;
        timestamp = timestamp.substr(1);
        q.timestamp = std::stoi(timestamp);
    }
    return q;
}
