#include <cstdint>
#include <unity.h>
#include <iostream>
#include "../msp_native/mock_serial.h"

#include "devCRSF.h"

using namespace std;
// Mock out the serial port using a string stream
std::string buf;
StringStream ss(buf);

// Create a CRSF object to test,
// using the StringStream as a mock UART
CRSF crsf(&ss);

GENERIC_CRC8 test_crc(CRSF_CRC_POLY);

void test_ver_to_u32(void)
{
    constexpr struct tagVerItem {
        const char verStr[128];
        const uint32_t verU32;
    } VERSION_STRINGS[] = {
        {{108,117,97,45,102,111,108,100,101,114,45,117,112,100,97,116,101,32,73,83,77,50,71,52,0}, 0}, // lua-folder-update ISM2G4
        {{0x32, 0x2e, 0x32, 0x2e, 0x31, 0x35, 32,73,83,77,50,71,52,0}, 0x0002020f}, // 2.2.15 ISM2G4
        {{0x31, 0x2e, 0x32, 0x2e, 0x33, 0x2e, 0x34, 32,73,83,77,50,71,52,0}, 0x01020304}, // 1.2.3.4 ISM2G4
        {{0x31, 0x30, 0x30, 0x2e, 0x32, 0x35, 0x35, 32,0}, 0x000064ff}, // 100.255(space)
        {{0}, 0},
    };

    const struct tagVerItem *ver = VERSION_STRINGS;
    while (ver->verStr[0])
    {
        TEST_ASSERT_EQUAL_HEX32(ver->verU32, CRSF::VersionStrToU32(ver->verStr));
        ++ver;
    }
}

void test_device_info(void)
{
    uint8_t deviceInformation[DEVICE_INFORMATION_LENGTH];

    TEST_ASSERT_EQUAL(22, DEVICE_INFORMATION_PAYLOAD_LENGTH);
    TEST_ASSERT_EQUAL(28, DEVICE_INFORMATION_LENGTH);

    crsf.GetDeviceInformation(deviceInformation, 0);
    crsf.SetExtendedHeaderAndCrc(deviceInformation, CRSF_FRAMETYPE_DEVICE_INFO, DEVICE_INFORMATION_FRAME_SIZE, CRSF_ADDRESS_CRSF_RECEIVER, CRSF_ADDRESS_FLIGHT_CONTROLLER);

    crsf_ext_header_t *header = (crsf_ext_header_t *) deviceInformation;

    TEST_ASSERT_EQUAL(26, header->frame_size);
    TEST_ASSERT_EQUAL(CRSF_FRAMETYPE_DEVICE_INFO, header->type);
    TEST_ASSERT_EQUAL(CRSF_ADDRESS_FLIGHT_CONTROLLER, header->dest_addr);
    TEST_ASSERT_EQUAL(CRSF_ADDRESS_FLIGHT_CONTROLLER, header->device_addr);
    TEST_ASSERT_EQUAL(CRSF_ADDRESS_CRSF_RECEIVER, header->orig_addr);
    TEST_ASSERT_EQUAL(DEVICE_INFORMATION_FRAME_SIZE, header->frame_size);

    uint8_t *data = deviceInformation + sizeof(crsf_ext_header_t);
    uint8_t compare [] = {0x74, 0x65, 0x73, 0x74, 0x69, 0x6e, 0x67, 0x0, 0x45, 0x4c, 0x52, 0x53, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    for(int i = 0; i < DEVICE_INFORMATION_PAYLOAD_LENGTH; i++)
    {
        TEST_ASSERT_EQUAL(compare[i], data[i]);
    }

    TEST_ASSERT_EQUAL(test_crc.calc(&deviceInformation[2], DEVICE_INFORMATION_LENGTH-3), deviceInformation[DEVICE_INFORMATION_LENGTH - 1]);
}


int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_ver_to_u32);
    RUN_TEST(test_device_info);
    UNITY_END();

    return 0;
}
