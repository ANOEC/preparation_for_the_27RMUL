#pragma once

namespace algorithm::control::forward_feedback {
class ForwardFeedback {
public:
    ForwardFeedback() = default;
    ~ForwardFeedback() = default;

    float compute(float ref, float dt);

    void set_gain(float gain);

    float out() const;
    void reset();

private:
    struct {
        float gain_ = 0.0f;
    } param_;

    struct {
        float output_ = 0.0f;
        float last_ref_ = 0.0f;
        float ref_ = 0.0f;
        bool initialized_ = false;
    } state_;
};
}
