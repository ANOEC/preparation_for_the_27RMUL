#pragma once

namespace algorithm::math::wave::chirp {
class Chirp {
public:
    Chirp() = default;
    Chirp(float start_freq, float end_freq, float period, float amplitude, float offset);

    void set_param(float start_freq, float end_freq, float period, float amplitude, float offset);

    void compute(float dt);

    float get_output() const;
    
    bool is_finished() const;

    void reset();

private:
    struct {
        float start_freq_ = 0.0f;
        float end_freq_ = 0.0f;
        float period_ = 0.0f;
        float amplitude_ = 0.0f;
        float offset_ = 0.0f;
    } param_;

    struct {
        float output_ = 0.0f;
        float current_phase_ = 0.0f;
        float current_time_ = 0.0f;
        bool finished_ = false;
    } status_;
};
}
