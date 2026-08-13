#include "alg_mecanum.h"
#include "alg_omni.h"
#include "alg_swerve.h"
#include <assert.h>
#include <math.h>

static int near(float a, float b) { return fabsf(a - b) < 1.0e-4F; }

int main(void)
{
    alg_mecanum_t mecanum;
    const alg_mecanum_config_t mc = {
        .wheel_radius_m=.1F, .half_wheelbase_m=.2F, .half_track_width_m=.2F,
        .direction_sign={1,1,1,1}, .odometry_weight={1,1,1,1},
        .maximum_wheel_angular_velocity_rad_per_s=100, .roller_arrangement=ALG_MECANUM_ROLLER_X};
    alg_chassis_velocity_t v = {.velocity_x_m_per_s=1.0F};
    bool available[4] = {true,true,true,true};
    float wheel[4], scale;
    assert(alg_mecanum_init(&mecanum, &mc) == ALG_CHASSIS_STATUS_OK);
    assert(alg_mecanum_inverse(&mecanum, &v, available, wheel, &scale) == ALG_CHASSIS_STATUS_OK);
    for (unsigned i=0;i<4;i++) assert(near(fabsf(wheel[i]), 10.0F));

    alg_omni_wheel_config_t ow[3];
    alg_omni_t omni;
    assert(alg_omni_configure_tangential_layout(ow, 3, .2F, .1F, 0, 1, NULL, 1) == ALG_CHASSIS_STATUS_OK);
    assert(alg_omni_init(&omni, ow, 3, 100) == ALG_CHASSIS_STATUS_OK);
    assert(alg_omni_inverse(&omni, &(alg_chassis_velocity_t){0}, NULL, wheel, 4, &scale) == ALG_CHASSIS_STATUS_OK);
    for (unsigned i=0;i<3;i++) assert(near(wheel[i], 0));

    alg_swerve_module_geometry_t geometry[4] = {{.2F,.2F},{.2F,-.2F},{-.2F,.2F},{-.2F,-.2F}};
    alg_swerve_t swerve;
    alg_swerve_module_target_t target[4];
    assert(alg_swerve_init(&swerve, geometry, 4, 5) == ALG_SWERVE_STATUS_OK);
    assert(alg_swerve_calculate(&swerve, &(alg_swerve_command_t){.velocity_x_m_per_s=1}, target, 4) == ALG_SWERVE_STATUS_OK);
    for (unsigned i=0;i<4;i++) assert(near(target[i].wheel_velocity_m_per_s, 1));
    return 0;
}
