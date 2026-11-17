#include "control_core.h"
#include <algorithm>
#include <cmath>
#include <new>
#include <Eigen/Core>

namespace {
struct Core {
    ControlConfig config;
    double integral = 0, previous_error = 0, previous_steer = 0, latched_brake = 0;
    bool has_previous = false;
    bool lqr = false;
    double wheelbase = 0;
    Eigen::Matrix2d p = Eigen::Matrix2d::Zero();
    Eigen::RowVector2d k = Eigen::RowVector2d::Zero();
};
bool solve_lqr(Core& core, double speed, double wheelbase, double dt, double weight) {
    if (!std::isfinite(speed) || !std::isfinite(wheelbase) || !std::isfinite(dt) ||
        !std::isfinite(weight) || std::min({speed,wheelbase,dt,weight})<=0) return false;
    Eigen::Matrix2d a, q;
    a << 1., speed*dt, 0., 1.;
    q << weight, 0., 0., 1.;
    const Eigen::Vector2d b(speed*speed*dt*dt/(2*wheelbase),speed*dt/wheelbase);
    Eigen::Matrix2d p=q;
    for (int iteration=0; iteration<10000; ++iteration) {
        const Eigen::RowVector2d k=(b.transpose()*p*a)/(1.+(b.transpose()*p*b)(0));
        const Eigen::Matrix2d next=q+a.transpose()*p*(a-b*k);
        if (!next.allFinite()) return false;
        if ((next-p).cwiseAbs().maxCoeff()<1e-10) {
            core.p=next;
            core.k=(b.transpose()*next*a)/(1.+(b.transpose()*next*b)(0));
            core.lqr=true;
            core.wheelbase=wheelbase;
            return core.k.allFinite();
        }
        p=next;
    }
    return false;
}
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
    core->integral=core->previous_error=core->previous_steer=core->latched_brake=0;
    core->has_previous=false;
}
static int step(void* handle, const ControlInput* input, double curvature, ControlOutput* output) {
    if (!handle || !input || !output || !valid_input(*input)) return 1;
    auto& c = *static_cast<Core*>(handle);
    if (c.lqr && !std::isfinite(curvature)) return 1;
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
    double angle = std::atan2(std::sin(i.path_heading-i.heading),
                                   std::cos(i.path_heading-i.heading))
                       - std::atan2(cfg.stanley_gain*i.cross_track,std::max(i.speed,.1));
    if (c.lqr) {
        const double heading_error=std::atan2(std::sin(i.heading-i.path_heading),
                                               std::cos(i.heading-i.path_heading));
        angle=std::atan(c.wheelbase*curvature)-c.k(0)*i.cross_track-c.k(1)*heading_error;
    }
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
int control_step(void* handle, const ControlInput* input, ControlOutput* output) {
    return step(handle,input,0.,output);
}
void* control_create_lqr(const ControlConfig* config, double speed, double wheelbase,
                         double dt, double position_weight) {
    auto* core=static_cast<Core*>(control_create(config));
    if (core && solve_lqr(*core,speed,wheelbase,dt,position_weight)) return core;
    delete core;
    return nullptr;
}
int control_step_lqr(void* handle, const ControlInput* input, double curvature, ControlOutput* output) {
    if (!handle || !static_cast<Core*>(handle)->lqr) return 1;
    return step(handle,input,curvature,output);
}
int control_lqr_solution(void* handle, double* gain, double* riccati) {
    if (!handle || !gain || !riccati) return 1;
    const auto& core=*static_cast<Core*>(handle);
    if (!core.lqr) return 1;
    for (int row=0; row<2; ++row) {
        gain[row]=core.k(row);
        for (int col=0; col<2; ++col) riccati[2*row+col]=core.p(row,col);
    }
    return 0;
}
void control_destroy(void* handle) {
    delete static_cast<Core*>(handle);
}
}
