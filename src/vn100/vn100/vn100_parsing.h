#pragma once
#include <string>
#include <vector>
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
namespace VN100BINARY {
    struct VN100_group1{

        using TimeStartupType=u_int64_t;
        using TimeSyncInType=u_int64_t;
        using YawPitchRollType=std::array<float,3>;
        using QuaternionType=std::array<float,4>;
        using AngularRateType=std::array<float,3>;
        using AccelType=std::array<float,3>;
        using ImuType=std::array<float,6>;
        using MagePressType=std::array<float,5>;
        using DeltaThetaType=std::array<float,7>;
        using VPEStatusType=u_int16_t;
        using SyncInCntType=u_int32_t;

        TimeStartupType TimeStartup;
        TimeSyncInType TimeSyncIn;
        YawPitchRollType YawPitchRoll;
        QuaternionType Quaternion;
        AngularRateType AngularRate;
        AccelType Accel;
        ImuType Imu;
        MagePressType MagPres;
        DeltaThetaType DeltaTheta;
        VPEStatusType VPEStatus;
        SyncInCntType SyncInCnt;

        bool Has_TimeStartup{false};
        bool Has_TimeSyncIn{false};
        bool Has_YawPitchRoll{false};
        bool Has_Quaternion{false};
        bool Has_AngularRate{false};
        bool Has_Accel{false};
        bool Has_Imu{false};
        bool Has_MagPres{false};
        bool Has_DeltaTheta{false};
        bool Has_VPEStatus{false};
        bool Has_SyncInCnt{false};


    };

    const std::array<size_t, 16> VN100_group1_len{
            8,  // TimeStartup,0
            0,  // res
            8,  // TimeSyncIn,2
            12, // YPR,3
            16, // Quat,4
            12, // Rate,5
            0,  // Res,6
            0 , // Rate,7
            12, // Accel,8
            24, // Raw Imu,9
            20, // MagPres,10
            28, // DeltaThetaVel,11
            2,  // VpeStatus,12
            0,  // Res,13
            8,  // SyncInCnt,14
            0 };

    // According to Table 19 - Binary Group 1
//    const static uint16_t group_1_TimeStartup_bit  = 1<<0;
//    const static uint16_t group_1_TimeSyncIn_bit   = 1<<2;
//    const static uint16_t group_1_YawPitchRoll_bit = 1<<3;
//    const static uint16_t group_1_Quaternion_bit   = 1<<4;
//    const static uint16_t group_1_AngularRate_bit  = 1<<5;
//    const static uint16_t group_1_Accel_bit        = 1<<8;
//    const static uint16_t group_1_Imu_bit          = 1<<9;
//    const static uint16_t group_1_MagPres_bit      = 1<<10;
//    const static uint16_t group_1_DeltaTheta_bit   = 1<<11;
//    const static uint16_t group_1_VpeStatus_bit    = 1<<12;
//    const static uint16_t group_1_SyncInCnt_bit    = 1<<13;

    const static size_t group_1_TimeStartup_ID   = 0;
    const static size_t group_1_TimeSyncIn_ID    = 2;
    const static size_t group_1_YawPitchRoll_ID  = 3;
    const static size_t group_1_Quaternion_ID    = 4;
    const static size_t group_1_AngularRate_ID   = 5;
    const static size_t group_1_Accel_ID         = 8;
    const static size_t group_1_Imu_ID           = 9;
    const static size_t group_1_MagPres_ID       = 10;
    const static size_t group_1_DeltaTheta_ID    = 11;
    const static size_t group_1_VPEStatus_ID     = 12;
    const static size_t group_1_SyncInCnt_ID     = 13;


    size_t getGroup1FieldOffset(uint16_t group, size_t selectedField);
    bool   getGroup1FieldPresent(uint16_t group, size_t selectedField);


    bool parse(const std::vector<uint8_t>& data, VN100_group1& parsed);

}
