#pragma once

#include <cstdint>

namespace algorithm::verify {
class CrcCalculator {
public:
    static constexpr uint8_t CRC8_INIT_NUM = 0xff;
    static constexpr uint16_t CRC16_INIT_NUM = 0xffff;

    static uint8_t GetCRC8CheckSum(const uint8_t* pch_message, uint16_t dw_length, uint8_t uc_crc8);

    static uint8_t VerifyCRC8CheckSum(const uint8_t* pch_message, uint16_t dw_length);

    static void AppendCRC8CheckSum(uint8_t* pch_message, uint16_t dw_length);

    static uint16_t GetCRC16CheckSum(const uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc);
    static uint8_t VerifyCRC16CheckSum(const uint8_t* pch_message, uint32_t dw_length);

    static void AppendCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length);
};
}