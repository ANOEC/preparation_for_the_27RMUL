#pragma once

#include <cstddef>
#include <cstdint>
namespace algorithm::verify {
class Checksum {
public:
    static uint8_t GetSumCheckSum(const uint8_t* pch_message, std::size_t data_len);
    static uint16_t GetSumCheckSum16(const uint8_t* pch_message, std::size_t data_len);

    static bool VerifySumCheckSum(const uint8_t* pch_message, std::size_t data_len, uint8_t checksum);
    static bool VerifySumCheckSum16(const uint8_t* pch_message, std::size_t data_len, uint16_t checksum);
    
    static bool VerifySumCheckSum(const uint8_t* pch_message, std::size_t data_len);
    static bool VerifySumCheckSum16BE(const uint8_t* pch_message, std::size_t data_len);
    static bool VerifySumCheckSum16LE(const uint8_t* pch_message, std::size_t data_len);

    static void AppendSumCheckSum(uint8_t* pch_message, std::size_t data_len);
    static void AppendSumCheckSum16BE(uint8_t* pch_message, std::size_t data_len);
    static void AppendSumCheckSum16LE(uint8_t* pch_message, std::size_t data_len);
};
}