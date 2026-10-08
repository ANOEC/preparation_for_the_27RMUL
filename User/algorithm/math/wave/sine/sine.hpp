#pragma once

#include "utility/user_assert.hpp"
#include <cmath>
#include <numbers>
namespace algorithm::math::wave {
class SineWaveGenerator {
public:
    SineWaveGenerator() = default;
    SineWaveGenerator(float frequency, float amplitude = std::numbers::pi_v<float>, float offset = 0.0)
        : output_(0.0f)
        , param_ {
            2.0f * std::numbers::pi_v<float> * frequency, amplitude, offset
        }
    {
        assert_always(frequency >= 0.0f);
        assert_always(amplitude >= 0.0f);
    }

    // 更新角度并返回正弦值
    float update(float current_time);

    // 设置正弦波的频率
    void set_frequency(float frequency);

    void set_amplitude(float amp);

    void set_offset(float off);

    inline void clear()
    {
        output_ = 0.0;
        time_ = 0.0;
        initialized_ = false;
    }

    inline float get_output() const
    {
        return output_;
    }

    inline float get_output_in_degrees() const
    {
        constexpr float radians_to_degrees = 180.0f / std::numbers::pi_v<float>;
        return output_ * radians_to_degrees;
    }

private:
    float output_;
    struct {
        float angularFrequency; // 角频率（每秒钟转过的弧度数）
        float amplitude; // 正弦波的振幅
        float offset; // 正弦波的偏移量
    } param_;

    float time_;
    bool initialized_ = false;
};
}
