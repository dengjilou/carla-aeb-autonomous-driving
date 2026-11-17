#include <rclcpp/rclcpp.hpp>
#include "control_runtime/msg/control_state.hpp"
#include "control_runtime/msg/control_command.hpp"
#include "control_runtime/msg/lqr_state.hpp"
#include "control_core.h"

class ControlNode : public rclcpp::Node {
    using State = control_runtime::msg::ControlState;
    using Command = control_runtime::msg::ControlCommand;
    using LqrState = control_runtime::msg::LqrState;
    void* core_ = nullptr;
    std::uint64_t session_ = 0, sequence_ = 0;
    bool has_result_ = false;
    Command cached_;
    rclcpp::Publisher<Command>::SharedPtr publisher_;
    rclcpp::Subscription<State>::SharedPtr subscriber_;
    rclcpp::Subscription<LqrState>::SharedPtr lqr_subscriber_;

    void receive(const State& state, const LqrState* lqr=nullptr) {
        Command out;
        out.session=state.session;
        out.sequence=state.sequence;
        out.stamp=state.stamp;
        if (core_ && state.session==session_ && has_result_ && state.sequence==sequence_) {
            publisher_->publish(cached_);
            return;
        }
        if (!core_ || state.session!=session_) {
            if (state.sequence!=0) { out.status=2; publisher_->publish(out); return; }
            control_destroy(core_);
            const ControlConfig config{state.gap_gain,state.relative_gain,state.standstill_gap,
                state.headway,state.stanley_gain,state.steer_limit,state.steer_alpha};
            core_=lqr ? control_create_lqr(&config,lqr->design_speed,lqr->wheelbase,
                                           lqr->design_dt,lqr->position_weight) : control_create(&config);
            session_=state.session;
            has_result_=false;
        }
        if (!core_ || (!has_result_ && state.sequence!=0) ||
            (has_result_ && state.sequence!=sequence_+1)) {
            out.status=2;
            publisher_->publish(out);
            return;
        }
        const ControlInput input{state.dt,state.speed,state.cruise,state.gap,state.closing,
            state.heading,state.path_heading,state.cross_track,state.max_angle,state.aeb_request,
            state.has_target?1:0,state.acc_enabled?1:0};
        ControlOutput result{};
        out.status=lqr ? control_step_lqr(core_,&input,lqr->curvature,&result) : control_step(core_,&input,&result);
        if (!out.status) {
            out.reference=result.reference; out.throttle=result.throttle;
            out.brake=result.brake; out.steer=result.steer; out.latched_brake=result.latched_brake;
            sequence_=state.sequence; has_result_=true; cached_=out;
        }
        publisher_->publish(out);
    }
public:
    ControlNode():Node("control_node") {
        publisher_=create_publisher<Command>("/control/command",rclcpp::QoS(1).reliable());
        subscriber_=create_subscription<State>("/control/state",rclcpp::QoS(1).reliable(),
            [this](State::ConstSharedPtr state){ receive(*state); });
        lqr_subscriber_=create_subscription<LqrState>("/control/lqr_state",rclcpp::QoS(1).reliable(),
            [this](LqrState::ConstSharedPtr state){ receive(state->state,state.get()); });
    }
    ~ControlNode() override { control_destroy(core_); }
};

int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
}
