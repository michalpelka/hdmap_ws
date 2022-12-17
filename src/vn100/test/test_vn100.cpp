#include "gtest/gtest.h"
#include "../clients/vn100/vn100_parsing.h"
#include <array>
#include <string>
using std::string;

const char *sampleVN100  = "$VNQMR,+0.004183,+0.001149,+0.089285,+0.995997,+00.0113,-00.2384,+00.1800,+00.020,-00.092,-09.683,-00.000272,+00.000824,+00.000073,T0000000543*3D";
const char *sampleVN100Cheskumed  = "VNQMR,+0.004183,+0.001149,+0.089285,+0.995997,+00.0113,-00.2384,+00.1800,+00.020,-00.092,-09.683,-00.000272,+00.000824,+00.000073,T0000000543";
const unsigned char sampleVN100checksum = 0x3D;




TEST(VN100_parsing, getCheckSumDataTest) {
    auto data = vn100_parsing::getCheckSumData(sampleVN100);
    EXPECT_STREQ(data.c_str(), sampleVN100Cheskumed);
}

TEST(VN100_parsing, computeChecksum) {
    auto data = vn100_parsing::getCheckSumData(sampleVN100);
    auto checksum = vn100_parsing::calculateChecksum((unsigned char*)data.c_str(), data.length());
    EXPECT_EQ(checksum, sampleVN100checksum);
}

TEST(VN100_parsing, computeData) {
    auto checksum = vn100_parsing::calculateChecksum((unsigned char*)sampleVN100Cheskumed, strlen(sampleVN100Cheskumed));
    EXPECT_EQ(checksum, sampleVN100checksum);
}

TEST(VN100_parsing, getChecksumFromData) {
    auto checksum = vn100_parsing::getChecksum(sampleVN100);
    EXPECT_EQ(checksum, sampleVN100checksum);
}

TEST(VN100_parsing, parseData) {
    auto vnqmr_data = vn100_parsing::vnqmr_parse(sampleVN100Cheskumed);
    EXPECT_NEAR(vnqmr_data.q[0], 0.004183, 0.0001);
    EXPECT_NEAR(vnqmr_data.q[1], 0.001149, 0.0001);
    EXPECT_NEAR(vnqmr_data.q[2], 0.089285, 0.0001);
    EXPECT_NEAR(vnqmr_data.q[3], 0.995997, 0.0001);

    EXPECT_NEAR(vnqmr_data.mag[0], 0.0113, 0.0001);
    EXPECT_NEAR(vnqmr_data.mag[1], -0.2384, 0.0001);
    EXPECT_NEAR(vnqmr_data.mag[2], 0.1800, 0.0001);

    EXPECT_NEAR(vnqmr_data.accel[0], 0.020, 0.001);
    EXPECT_NEAR(vnqmr_data.accel[1], -0.092, 0.001);
    EXPECT_NEAR(vnqmr_data.accel[2], -9.683, 0.001);

    EXPECT_NEAR(vnqmr_data.gyro[0], -0.000272, 0.0001);
    EXPECT_NEAR(vnqmr_data.gyro[1],  0.000824, 0.0001);
    EXPECT_NEAR(vnqmr_data.gyro[2],  0.000073, 0.0001);

    EXPECT_NEAR(vnqmr_data.timestamp,  543, 0.01);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}