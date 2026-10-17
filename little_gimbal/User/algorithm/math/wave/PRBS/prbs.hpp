#pragma once

#include <cstdint>

namespace algorithm::math::wave::prbs {
class Prbs {
public:
    Prbs() = default;
    Prbs(float amplitude, float offset, float period, float update_interval);

    void set_param(float amplitude, float offset, float period, float update_interval);

    void compute(float dt);

    float get_output() const;

    bool is_finished() const;

    void reset();

private:
    struct {
        float amplitude_;
        float offset_;
        float period_; // 总测试时长
        float update_interval_; // PRBS 状态更新间隔 (时钟分频器)
    } param_;

    struct {
        float output_;
        float current_time_ = 0.0f;
        float time_since_last_update_ = 0.0f;
        uint16_t lfsr_state_ = 0x07FF; // 11-bit 线性反馈移位寄存器状态，绝对不能为0
        bool finished_ = false;
    } status_;
};
}