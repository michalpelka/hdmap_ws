#include "vn100_parsing.h"
#include "algorithm"
#include "sstream"
#include "assert.h"
#include <cstring>
#include <iostream>

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

size_t VN100BINARY::getGroup1FieldOffset(uint16_t group, size_t selectedField) {
    size_t offset = 0;
    for (size_t i = 0; i < selectedField; i++)
    {
        if (group & (1 << i)) {
            offset += VN100_group1_len[i];
        }
    }
    return offset;
}

bool  VN100BINARY::getGroup1FieldPresent(uint16_t group, size_t selectedField){
    return (group & (1 << selectedField));
}

bool VN100BINARY::parse(const std::vector<uint8_t>& data, VN100_group1& parsed) {
    if (data.size() < 9)
    {
        return false;
    }
    const uint8_t selectedGroups = static_cast<uint8_t>(data[1]);
    uint8_t group_count = 0 + selectedGroups && 0x01 + selectedGroups && (1 << 2) + selectedGroups && (1 << 4);

    assert(group_count == 1); // only one group is suported
    assert(selectedGroups && 0x01); // only common group is suported
    uint16_t group1_fields = data[3] << 8 | data[2];

    const auto payload_begin = data.begin() + 2 + 2 * group_count;

    parsed.Has_TimeStartup   = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_TimeStartup_ID);
    parsed.Has_TimeSyncIn    = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_TimeSyncIn_ID);
    parsed.Has_YawPitchRoll  = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_YawPitchRoll_ID);
    parsed.Has_Quaternion    = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_Quaternion_ID);
    parsed.Has_AngularRate   = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_AngularRate_ID);
    parsed.Has_Accel         = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_Accel_ID);
    parsed.Has_Imu           = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_Imu_ID);
    parsed.Has_MagPres       = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_MagPres_ID);
    parsed.Has_DeltaTheta    = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_DeltaTheta_ID);
    parsed.Has_VPEStatus     = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_VPEStatus_ID);
    parsed.Has_SyncInCnt     = VN100BINARY::getGroup1FieldPresent(group1_fields, VN100BINARY::group_1_SyncInCnt_ID);

    //TimeStartup
    if (parsed.Has_TimeStartup)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_TimeStartup_ID);
        std::memcpy(&parsed.TimeStartup, &(*ptr), sizeof(VN100_group1::TimeStartupType));
    }
    //TimeSyncIn
    if (parsed.Has_TimeSyncIn)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_TimeSyncIn_ID);
        std::memcpy(&parsed.TimeSyncIn, &(*ptr), sizeof(VN100_group1::TimeSyncInType));
    }
    //YawPitchRoll
    if (parsed.Has_YawPitchRoll)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_YawPitchRoll_ID);
        std::memcpy(parsed.YawPitchRoll.data(), &(*ptr), sizeof(float)*parsed.YawPitchRoll.size());
    }
    //Quaternion
    if (parsed.Has_Quaternion)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_Quaternion_ID);
        std::memcpy(parsed.Quaternion.data(), &(*ptr), sizeof(float)*parsed.Quaternion.size());
    }
    //AngularRate
    if (parsed.Has_AngularRate)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_AngularRate_ID);
        std::memcpy(parsed.AngularRate.data(), &(*ptr), sizeof(float)*parsed.AngularRate.size());
    }
    //Accel
    if (parsed.Has_Accel)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_Accel_ID);
        std::memcpy(parsed.Accel.data(), &(*ptr), sizeof(float)*parsed.Accel.size());
    }
    //Imu
    if (parsed.Has_Imu)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_Imu_ID);
        std::memcpy(parsed.Imu.data(), &(*ptr), sizeof(float)*parsed.Imu.size());
    }
    //MagPres
    if (parsed.Has_MagPres)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_MagPres_ID);
        std::memcpy(parsed.MagPres.data(), &(*ptr), sizeof(float)*parsed.MagPres.size());
    }
    //MagPres
    if (parsed.Has_DeltaTheta)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_DeltaTheta_ID);
        std::memcpy(parsed.DeltaTheta.data(), &(*ptr), sizeof(float)*parsed.DeltaTheta.size());
    }
    //VPEStatus
    if (parsed.Has_VPEStatus)
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_VPEStatus_ID);
        std::memcpy(&parsed.VPEStatus, &(*ptr), sizeof(VN100_group1::VPEStatusType));
    }
    //SyncInCnt
    if (parsed.Has_SyncInCnt )
    {
        auto ptr = payload_begin + VN100BINARY::getGroup1FieldOffset(group1_fields, VN100BINARY::group_1_SyncInCnt_ID);
        std::memcpy(&parsed.SyncInCnt , &(*ptr), sizeof(VN100_group1::SyncInCntType));
    }
    return true;
}

