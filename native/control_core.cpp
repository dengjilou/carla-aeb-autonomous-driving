#include "control_core.h"
#include <algorithm>
#include <cmath>
#include <new>

namespace {
struct Core {
    ControlConfig config;
    double integral = 0, previous_error = 0, previous_steer = 0, latched_brake = 0;
    bool has_previous = false;
};
bool valid_config(const ControlConfig& c) {
    return std::isfinite(c.gap_gain) && std::isfinite(c.relative_gain)
        && std::isfinite(c.standstill_gap) && std::isfinite(c.headway)
        && std::isfinite(c.stanley_gain) && std::isfinite(c.steer_limit)
        && std::isfinite(c.steer_alpha)
        && c.gap_gain >= 0 && c.relative_gain >= 0 && c.standstill_gap >= 0
        && c.headway >= 0 && c.stanley_gain >= 0
        && c.steer_limit > 0 && c.steer_limit <= 1
        && c.steer_alpha > 0 && c.steer_alpha <= 1;
}
bool valid_input(const ControlInput& i) {
    return std::isfinite(i.dt) && std::isfinite(i.speed) && std::isfinite(i.cruise)
        && (!i.has_target || (std::isfinite(i.gap) && std::isfinite(i.closing)))
        && std::isfinite(i.heading) && std::isfinite(i.path_heading)
        && std::isfinite(i.cross_track) && std::isfinite(i.max_angle)
        && std::isfinite(i.aeb_request) && i.dt > 0 && i.speed >= 0
        && i.cruise >= 0 && i.max_angle > 0 && i.aeb_request >= 0 && i.aeb_request <= 1
        && (i.has_target == 0 || i.has_target == 1)
        && (i.acc_enabled == 0 || i.acc_enabled == 1);
}
}
extern "C" {
void* control_create(const ControlConfig* config) {
    if (!config || !valid_config(*config)) return nullptr;
    auto* core = new (std::nothrow) Core;
    if (core) core->config = *config;
    return core;
}
void control_reset(void* handle) {
    if (!handle) return;
    auto* core = static_cast<Core*>(handle);
    *core = Core{core->config};
}
int control_step(void* handle, const ControlInput* input, ControlOutput* output) {
    if (!handle || !input || !output || !valid_input(*input)) return 1;
    auto& c = *static_cast<Core*>(handle);
    const auto& i = *input;
    const auto& cfg = c.config;
    double reference = i.cruise;
    if (i.acc_enabled && i.has_target) {
        const double desired = cfg.standstill_gap + cfg.headway * i.speed;
        const double acceleration = std::clamp(cfg.gap_gain*(i.gap-desired)
                                              - cfg.relative_gain*i.closing, -3., 2.);
        reference = std::clamp(i.speed+acceleration, 0., i.cruise);
    }
    const double error = reference-i.speed;
    const double derivative = c.has_previous ? (error-c.previous_error)/i.dt : 0.;
    const double candidate = std::clamp(c.integral+error*i.dt,-3.,3.);
    double command = .45*error + .12*candidate + .01*derivative;
    if (command >= -1. && command <= .75) c.integral = candidate;
    c.previous_error = error;
    c.has_previous = true;
    command = std::clamp(command,-1.,.75);
    double throttle = std::max(0.,command), brake = std::max(0.,-command);
    const double angle = std::atan2(std::sin(i.path_heading-i.heading),
                                   std::cos(i.path_heading-i.heading))
                       - std::atan2(cfg.stanley_gain*i.cross_track,std::max(i.speed,.1));
    const double raw_steer = angle/i.max_angle;
    c.previous_steer = cfg.steer_alpha*std::clamp(raw_steer,-cfg.steer_limit,cfg.steer_limit)
                    + (1-cfg.steer_alpha)*c.previous_steer;
    c.latched_brake = std::max(c.latched_brake,i.aeb_request);
    if (c.latched_brake > 0) {
        throttle = 0;
        brake = std::max(brake,c.latched_brake);
    }
    *output = ControlOutput{reference,throttle,brake,c.previous_steer,c.latched_brake};
    return 0;
}
void control_destroy(void* handle) {
    delete static_cast<Core*>(handle);
}
}
