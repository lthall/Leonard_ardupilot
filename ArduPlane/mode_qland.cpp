#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLand::_enter()
{
    plane.mode_qloiter._enter();
    // Landing must start from a freshly initialised horizontal controller. ModeQLoiter::_enter()
    // uses init_target(false), which preserves an already-running controller for smooth in-air
    // mode changes; that is wrong for a landing, where any wind-fighting integrator windup or an
    // unreachable target carried over from QLOITER (e.g. after a forward-motor failure) would be
    // preserved into the descent and drive the vehicle off position. Force a full re-init.
    loiter_nav->init_target(true);
    quadplane.throttle_wait = false;
    quadplane.setup_target_position();
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    quadplane.last_land_final_agl_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;
#if AP_LANDINGGEAR_ENABLED
    plane.g2.landing_gear.deploy_for_landing();
#endif

    return true;
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}

void ModeQLand::run()
{
    /*
      use QLOITER to do the main control
     */
    plane.mode_qloiter.run();
}

#endif
