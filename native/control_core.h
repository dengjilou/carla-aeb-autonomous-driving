#pragma once
#include <cstdint>

extern "C" {
struct ControlConfig {
    double gap_gain, relative_gain, standstill_gap, headway;
    double stanley_gain, steer_limit, steer_alpha;
};
struct ControlInput {
    double dt, speed, cruise, gap, closing;
    double heading, path_heading, cross_track, max_angle, aeb_request;
    std::int32_t has_target, acc_enabled;
};
struct ControlOutput {
    double reference, throttle, brake, steer, latched_brake;
};
void* control_create(const ControlConfig* config);
void control_reset(void* handle);
int control_step(void* handle, const ControlInput* input, ControlOutput* output);
void control_destroy(void* handle);
}
