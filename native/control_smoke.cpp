#include "control_core.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
    const ControlConfig config{.3,.4,8,1.8,2,.7,.4};
    void* core = control_create(&config);
    if (!core) return EXIT_FAILURE;
    ControlInput input{.05,10,12,26,0,0,0,0,1.2,0,1,1};
    ControlOutput out{};
    bool ok = control_step(core,&input,&out)==0 && std::abs(out.reference-10)<1e-12;
    input.aeb_request = .4;
    ok = ok && control_step(core,&input,&out)==0 && out.throttle==0 && out.brake>=.4;
    input.aeb_request = 0;
    ok = ok && control_step(core,&input,&out)==0 && out.brake>=.4;
    control_reset(core);
    ok = ok && control_step(core,&input,&out)==0 && out.latched_brake==0;
    input.dt = 0;
    ok = ok && control_step(core,&input,&out)!=0;
    input.dt = .05;
    ok = ok && control_step(core,&input,&out)==0 && std::abs(out.reference-10)<1e-12;
    control_destroy(core);
    std::cout << (ok ? "Control smoke passed\n" : "Control smoke failed\n");
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
