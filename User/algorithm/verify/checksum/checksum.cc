#include "./checksum.hpp"

namespace {
template <typename T>
T SumBytes(const uint8_t* data, std::size_t len)
{
    T sum = 0;
    for (std::size_t i = 0; i < len; ++i)
        sum += data[i];
    return sum;
}

}

namespace algorithm::verify {

uint8_t Checksum::GetSumCheckSum(const uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr)
        return 0;
    return SumBytes<uint8_t>(pch_message, data_len);
}

uint16_t Checksum::GetSumCheckSum16(const uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr)
        return 0;
    return SumBytes<uint16_t>(pch_message, data_len);
}

bool Checksum::VerifySumCheckSum(const uint8_t* pch_message, std::size_t data_len, uint8_t checksum)
{
    if (pch_message == nullptr || data_len < 1)
        return false;
    return GetSumCheckSum(pch_message, data_len) == checksum;
}

bool Checksum::VerifySumCheckSum16(const uint8_t* pch_message, std::size_t data_len, uint16_t checksum)
{
    if (pch_message == nullptr || data_len < 1)
        return false;
    return GetSumCheckSum16(pch_message, data_len) == checksum;
}

bool Checksum::VerifySumCheckSum(const uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr || data_len < 1)
        return false;
    return GetSumCheckSum(pch_message, data_len - 1) == pch_message[data_len - 1];
}

bool Checksum::VerifySumCheckSum16BE(const uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr || data_len < 2)
        return false;
    const uint16_t sum = GetSumCheckSum16(pch_message, data_len - 2);

    return ((sum >> 8) & 0xFF) == pch_message[data_len - 2]
        && (sum & 0xFF) == pch_message[data_len - 1];
}

bool Checksum::VerifySumCheckSum16LE(const uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr || data_len < 2)
        return false;
    const uint16_t sum = GetSumCheckSum16(pch_message, data_len - 2);

    return (sum & 0xFF) == pch_message[data_len - 2]
        && ((sum >> 8) & 0xFF) == pch_message[data_len - 1];
}

void Checksum::AppendSumCheckSum(uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr || data_len < 1)
        return;
    pch_message[data_len - 1] = GetSumCheckSum(pch_message, data_len - 1);
}

void Checksum::AppendSumCheckSum16BE(uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr || data_len < 2)
        return;
    const uint16_t sum = GetSumCheckSum16(pch_message, data_len - 2);

    pch_message[data_len - 2] = (sum >> 8) & 0xFF;
    pch_message[data_len - 1] = sum & 0xFF;
}

void Checksum::AppendSumCheckSum16LE(uint8_t* pch_message, std::size_t data_len)
{
    if (pch_message == nullptr || data_len < 2)
        return;
    const uint16_t sum = GetSumCheckSum16(pch_message, data_len - 2);

    pch_message[data_len - 2] = sum & 0xFF;
    pch_message[data_len - 1] = (sum >> 8) & 0xFF;
}

}