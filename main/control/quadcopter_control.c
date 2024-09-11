#include "quadcopter_control.h"
#include "filters.h"
#include "math.h"
#include "typedefs.h"
#include "storage/nv_storage.h"
#include "setup.h"
#include "gpio.h"
#include "quaternion.h"

#define BENCH_MODE 0

vector2_t leash_vector = {0.0f, 0.0f};
leash_t target_leash_location;
static float velocity_from_distance = 0;

static radio_control_t *radio_p;
static flight_t *flight_p;
static target_t *target_p;
static states_t *state_p;
static telemetry_t *telemetry_p;
static config_t *config_p;
static waypoint_t *waypoint_p;
static gnss_t *gnss_ptr;
static home_point_t home;

struct PID
{
    float errPitch, errRoll, errYaw;
    float errPitchPrev, errRollPrev, errYawPrev;
    float pitchDegsPrev, rollDegsPrev, yawDegsPrev;
    float pitchPout, rollPout, yawPout;
    float pitchIout, rollIout, yawIout;
    float pitchDout, rollDout, yawDout;
    float pitch_ff_out, roll_ff_out, yaw_ff_out;
    float pitchPIDout, rollPIDout, yawPIout;
    float errVel_x, errVel_x_prev, velocity_x_ms_prev;
    float errVel_y, errVel_y_prev, velocity_y_ms_prev;
    float errVel_z, errVel_z_prev, velocity_z_ms_prev;
    float altPout, altIout, altDout;
    float posXPout, posXIout;
    float posYPout, posYIout;
};
static struct PID pid;
struct throttle
{
    float m1, m2, m3, m4;
};

static struct throttle thr;
static biquad_lpf_t lpf_pitch_d_term;
static biquad_lpf_t lpf_roll_d_term;
static biquad_lpf_t lpf_yaw_p_term;
static target_nav_t target_navigation_leash;
static target_nav_t target_navigation_wp;

static uint8_t use_gps_hold = 0;
static uint8_t is_hold_location_set = 0;
static uint8_t disarmed_by_landing = 0;
static uint8_t is_leash_at_the_target = 0;
static float prev_target_navigation_distance_cm = 1000000.0f; //(has to be something big)

static float course_correction = 0;

static int16_t apply_deadband(int16_t input, uint16_t deadband);
static uint8_t navigation_controller();
static uint8_t outer_control_loop_rth();
static uint8_t outer_control_loop_wp();
static void quadcopter_flight_mode_control();
static void outer_control_loop_rc(uint8_t land_flag);
static void inner_control_loop();
static void arm();
static void disarm();
static void get_distance_bearing(target_nav_t *tar_nav, int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
static uint8_t landing_detector(uint8_t need_reset);
static float calculate_target_yaw_degs_from_target_heading_deg();
static float calculate_target_pitch_degs_from_target_pitch_deg();
static float calculate_target_roll_degs_from_target_roll_deg();
static void navigation_start(int32_t latitude, int32_t longitude);
static uint8_t is_leash_left_behind();

void quadcopter_control_init(radio_control_t *rc, telemetry_t *tlm, flight_t *flt, target_t *trg, states_t *stt, config_t *cfg, waypoint_t *wp, gnss_t *gnss)
{
    radio_p = rc;
    flight_p = flt;
    target_p = trg;
    state_p = stt;
    telemetry_p = tlm;
    config_p = cfg;
    waypoint_p = wp;
    gnss_ptr = gnss;

    biquad_lpf_configure(D_TERM_CUTOFF_FREQ, 1000.0f, &lpf_pitch_d_term);
    biquad_lpf_configure(D_TERM_CUTOFF_FREQ, 1000.0f, &lpf_roll_d_term);
    biquad_lpf_configure(P_YAW_CUTOFF_FREQ, 1000.0f, &lpf_yaw_p_term);
}

static void quadcopter_flight_mode_control()
{
    // ||==============================================||
    // ||                 RC ARM LOGIC                 ||
    // ||==============================================||
    if (ARM_ON_CONDITION(radio_p->channel[RC_ARM_CH]) && flight_p->arm_status == 0 && disarmed_by_landing == 0)
    {
        // if alt hold is on we want throttle stick in the middle else zero
        if ((flight_p->alt_hold_status == 1 && (radio_p->channel[RC_THROTTLE_CH] < 1600 && radio_p->channel[RC_THROTTLE_CH] > 1400)) || (flight_p->alt_hold_status == 0 && radio_p->channel[RC_THROTTLE_CH] < 1100))
        {
            // if RTH stick is not active
            if (!RTH_ON_CONDITION(radio_p->channel[RC_RTH_CH]))
            {
                arm();
            }
        }
    }
    else if (!ARM_ON_CONDITION(radio_p->channel[RC_ARM_CH]) && (flight_p->arm_status == 1 || disarmed_by_landing == 1))
    {
        disarm();
        disarmed_by_landing = 0;
        /////////////////////////////////
        //    RESET OTHER FUNCTIONS    // 
        flight_p->alt_hold_status = 0;
        flight_p->pos_hold_status = 0;
        use_gps_hold = 0;
        is_hold_location_set = 0;
        flight_p->waypoint_mission_status = 0;

        storage_save(waypoint_p, MISSION_DATA);

        target_p->latitude = gnss_ptr->latitude;
        target_p->longitude = gnss_ptr->longitude;
        target_p->altitude = state_p->altitude_m;
        /////////////////////////////////
    }
    // ||==============================================||
    // ||              RC ALT HOLD LOGIC               ||
    // ||==============================================||
    if ((ALT_HOLD_ON_CONDITION(radio_p->channel[RC_ALT_HOLD_CH]) && flight_p->alt_hold_status == 0) && flight_p->rth_status == 0)
    {
        flight_p->alt_hold_status = 1;
        pid.altIout = target_p->throttle;
        target_p->altitude = state_p->altitude_m;
    }
    else if ((!ALT_HOLD_ON_CONDITION(radio_p->channel[RC_ALT_HOLD_CH]) && flight_p->alt_hold_status == 1) && flight_p->rth_status == 0)
    {
        flight_p->alt_hold_status = 0;
    }

    // ||==============================================||
    // ||              RC POS HOLD LOGIC               ||
    // ||==============================================||
    if ((POS_HOLD_ON_CONDITION(radio_p->channel[RC_POS_HOLD_CH]) &&
         flight_p->pos_hold_status == 0) && flight_p->rth_status == 0)
    {
        flight_p->pos_hold_status = 1;
        if (telemetry_p->is_gnss_sanity_check_ok == 1)
        {
            target_p->latitude = gnss_ptr->latitude;
            target_p->longitude = gnss_ptr->longitude;
            use_gps_hold = 1;
            is_hold_location_set = 1;
        }
    }
    else if ((!POS_HOLD_ON_CONDITION(radio_p->channel[RC_POS_HOLD_CH]) && flight_p->pos_hold_status == 1) && flight_p->rth_status == 0)
    {
        flight_p->pos_hold_status = 0;
        use_gps_hold = 0;
        is_hold_location_set = 0;
    }

    // ||==============================================||
    // ||              RC WAYPOINT LOGIC               ||
    // ||==============================================||
    if ((WAYPOINT_ON_CONDITION(radio_p->channel[RC_WAYPOINT_CH])) &&
        flight_p->waypoint_mission_status == 0 &&
        flight_p->arm_status == 1 &&
        flight_p->pos_hold_status == 1 &&
        flight_p->alt_hold_status == 1 &&
        flight_p->is_takeoff_done == 1 &&
        flight_p->rth_status == 0 &&
        telemetry_p->is_gnss_sanity_check_ok == 1)
    {

        if (waypoint_p->counter == 0 && (waypoint_p->latitude[0] != 0 && waypoint_p->longitude[0] != 0 && waypoint_p->altitude[0] != 0))
        {
            target_p->latitude = waypoint_p->latitude[0];
            target_p->longitude = waypoint_p->longitude[0];
            target_p->altitude = waypoint_p->altitude[0] / 10.0f;
        }
        else if (waypoint_p->counter > 0 && waypoint_p->is_reached == 0 && (waypoint_p->latitude[waypoint_p->counter-1] != 0 && waypoint_p->longitude[waypoint_p->counter-1] != 0 && waypoint_p->altitude[waypoint_p->counter-1] != 0))
        {
            target_p->latitude = waypoint_p->latitude[waypoint_p->counter-1];
            target_p->longitude = waypoint_p->longitude[waypoint_p->counter-1];
            target_p->altitude = waypoint_p->altitude[waypoint_p->counter-1] / 10.0f;
        }

        navigation_start(target_p->latitude, target_p->longitude);

        flight_p->waypoint_mission_status = 1;
        flight_p->is_rth_done = 0;
    }
    else if ((flight_p->waypoint_mission_status == 1 &&
              (!WAYPOINT_ON_CONDITION(radio_p->channel[RC_WAYPOINT_CH]) || flight_p->pos_hold_status == 0 || flight_p->alt_hold_status == 0)) &&
             flight_p->rth_status == 0)
    {
        flight_p->waypoint_mission_status = 0;
        target_p->latitude = gnss_ptr->latitude;
        target_p->longitude = gnss_ptr->longitude;
        target_p->altitude = state_p->altitude_m;
    }
    // ||==============================================||
    // ||                RC RTH LOGIC                  ||
    // ||==============================================||
    if (RTH_ON_CONDITION(radio_p->channel[RC_RTH_CH]) &&
        flight_p->rth_status == 0 &&
        flight_p->arm_status == 1)
    {
        flight_p->rth_status = 1;
        flight_p->is_rth_done = 0;
        flight_p->alt_hold_status = 1;
        pid.altIout = target_p->throttle;

        if (home.latitude != 0 && home.longitude != 0)
        {
            target_p->latitude = home.latitude;
            target_p->longitude = home.longitude;

            navigation_start(target_p->latitude, target_p->longitude);
        }
        else if (telemetry_p->is_gnss_sanity_check_ok == 1)
        {
            target_p->latitude = gnss_ptr->latitude;
            target_p->longitude = gnss_ptr->longitude;
            use_gps_hold = 1;
            is_hold_location_set = 1;
        }
        target_p->altitude = state_p->altitude_m;
        flight_p->pos_hold_status = 1;
    }
    else if (!RTH_ON_CONDITION(radio_p->channel[RC_RTH_CH]) && flight_p->rth_status == 1)
    {
        flight_p->rth_status = 0;

        if (flight_p->waypoint_mission_status == 1 && !(waypoint_p->is_reached == 1 && (waypoint_p->counter >= MAX_WP_COUNT || (waypoint_p->latitude[waypoint_p->counter] == 0 || waypoint_p->longitude[waypoint_p->counter] == 0))))
        {
            target_p->latitude = waypoint_p->latitude[waypoint_p->counter - 1];
            target_p->longitude = waypoint_p->longitude[waypoint_p->counter - 1];
            target_p->altitude = waypoint_p->altitude[waypoint_p->counter - 1] / 10.0f;

            navigation_start(target_p->latitude, target_p->longitude);
        }
        else if (POS_HOLD_ON_CONDITION(radio_p->channel[RC_POS_HOLD_CH]) && telemetry_p->is_gnss_sanity_check_ok == 1) //else if ((radio_p->ch6 > 1400 && radio_p->ch6 < 1600) && telemetry_p->is_gnss_sanity_check_ok == 1)
        {
            target_p->latitude = gnss_ptr->latitude;
            target_p->longitude = gnss_ptr->longitude;
            use_gps_hold = 1;
            is_hold_location_set = 1;
        }
        target_p->altitude = state_p->altitude_m;
    }
}

void quadcopter_flight_control() // 1000Hz
{
    static uint8_t counter = 0;
    counter++;
    if (counter > 100) // 10Hz
    {
        counter = 0;
        quadcopter_flight_mode_control();
    }


    if (flight_p->arm_status == 1)
    {
        // return to home function has highest priority
        if (flight_p->rth_status == 1)
        {
            // if home locaiton not set
            if (home.latitude == 0 || home.longitude == 0)
            {
                // regular control, autoland flag set
                outer_control_loop_rc(1);
            }
            else if (outer_control_loop_rth() == 0)
            {
                // home reached or gps not reliable,
                // switch to regular control, autoland flag set
                outer_control_loop_rc(1);
            }
        }
        else if (flight_p->waypoint_mission_status == 1)
        {
            static uint8_t prev_ret = 1;
            // ret = 0 --> gnss err or fall to RC control after end of mission
            // ret = 1 --> wp mission is ongoing
            // ret = 2 --> wp mission ended. rth without autolanding (origin is set)
            // ret = 3 --> wp mission ended. rth with autolanding (origin is set)
            uint8_t ret = outer_control_loop_wp();

            // check if end of mission behaviour is RTH 
            if ((prev_ret == 1 && ret == 2) || (prev_ret == 1 && ret == 3) || (prev_ret == 0 && ret == 2) || (prev_ret == 0 && ret == 3))
            {
                target_p->latitude = home.latitude;
                target_p->longitude = home.longitude;
                navigation_start(target_p->latitude, target_p->longitude);
            }
            prev_ret = ret;

            if (ret == 0)
            {
                // wp list ended or gps not reliable,
                // switch to regular control, autoland flag not set
                outer_control_loop_rc(0);
            }
            else if (ret == 2)
            {
                if (outer_control_loop_rth() == 0)
                {
                    outer_control_loop_rc(0);
                }
            }
            else if (ret == 3)
            {
                if (outer_control_loop_rth() == 0)
                {
                    outer_control_loop_rc(1);
                }
            }
        }
        else
        {
            // regular control, autoland not set
            outer_control_loop_rc(0);
        }

        inner_control_loop();
    }
    else
    {
        disarm();
    }
}

static uint8_t outer_control_loop_rth() // 1000 Hz
{
    if (flight_p->is_rth_done == 1 || telemetry_p->is_gnss_sanity_check_ok == 0)
        return 0;

    if (navigation_controller() == 1)
    {
        flight_p->is_rth_done = 1;
        return 0;
    }
    return 1;
}

static uint8_t outer_control_loop_wp() // 1000 Hz
{
    // gnss not available. Return early. Fall to manual RC control
    if (telemetry_p->is_gnss_sanity_check_ok == 0) return 0;
    // end of mission detected. select what to do next
    else if (waypoint_p->is_reached == 1 && (waypoint_p->counter >= MAX_WP_COUNT || (waypoint_p->latitude[waypoint_p->counter] == 0 || waypoint_p->longitude[waypoint_p->counter] == 0)))
    {
        if (waypoint_p->end_of_mission_behaviour == 0) return 0;                                                     // fall to RC manual control (passive position hold)
        else if (waypoint_p->end_of_mission_behaviour == 1 && home.latitude != 0 && home.longitude != 0) return 2;   // activate rth without autoland
        else if (waypoint_p->end_of_mission_behaviour == 2 && home.latitude != 0 && home.longitude != 0) return 3;   // activate rth with autoland
        else return 0;
    }

    // set initial or next wp location as target
    // waypoint_p->is_reached == 1 default value
    if (waypoint_p->is_reached == 1)
    {

        if (waypoint_p->counter == 0)
        {
            navigation_start(waypoint_p->latitude[0], waypoint_p->longitude[0]);
        }
        else
        {
            // if waypoint->counter > 0, then find the vector that points from current waypoint to next waypoint
            leash_vector.x = waypoint_p->latitude[waypoint_p->counter] - waypoint_p->latitude[waypoint_p->counter - 1];
            leash_vector.y = waypoint_p->longitude[waypoint_p->counter] - waypoint_p->longitude[waypoint_p->counter - 1];

            // normalize the vector that points to next waypoint. leash will go towards this direction
            norm_vector2(&leash_vector);
        }

        // leash will reach this waypoint eventually
        // waypoint altitude 25 means 2.5 meters so we divide by 10
        target_p->latitude = waypoint_p->latitude[waypoint_p->counter];
        target_p->longitude = waypoint_p->longitude[waypoint_p->counter];
        target_p->altitude = waypoint_p->altitude[waypoint_p->counter] / 10.0f;

        // reset waypoint_p->is_reached flag
        waypoint_p->is_reached = 0;
        // waypoint counter is increased by one before we reach that waypoint
        // eg. if waypoint_counter == 1, we are flying to waypoint 0
        waypoint_p->counter++;
    }
    else
    {
        // Leash and all targets are set. let's get there
        // navigation controller returns 0 if it didn't reach the target yet
        // returns 1 if target location reached
        if (navigation_controller() == 1)
        {
            waypoint_p->is_reached = 1;
            return 1;
        }
    }
    return 1;
}


// This function should only be called if gnss reliable
static uint8_t navigation_controller() // 100 Hz
{
    static float calculated_heading_correction = 0;
    static float gps_heading_wp_heading_diff = 0;
    static uint8_t counter = 0;

    counter++;
    if (counter >= 10)
    {
        counter = 0;

        // calculate distance/bearing to leash location from current location
        get_distance_bearing(&target_navigation_leash, target_leash_location.latitude, target_leash_location.longitude, gnss_ptr->latitude, gnss_ptr->longitude);
        get_distance_bearing(&target_navigation_wp, target_p->latitude, target_p->longitude, gnss_ptr->latitude, gnss_ptr->longitude);

        telemetry_p->target_latitude = target_p->latitude;
        telemetry_p->target_longitude = target_p->longitude;
        telemetry_p->distance_m_2d = target_navigation_wp.distance_cm / 100.0f;

        if (is_leash_at_the_target == 1)
        {
            if (target_navigation_leash.distance_cm < config_p->wp_threshold_cm)
            {
                // keep moving forward until we detect we are going past the location
                // in this phase we dont change heading_deg
                if (target_navigation_leash.distance_cm > prev_target_navigation_distance_cm)
                {
                    // we reached the target location
                    is_leash_at_the_target = 0;
                    prev_target_navigation_distance_cm = 100000.0f;
                    return 1;
                }
                // keep previous distance
                prev_target_navigation_distance_cm = target_navigation_leash.distance_cm;
            }
            else
            {
                // keep heading_deg towards leash location
                target_p->heading_deg = target_navigation_leash.bearing_deg + course_correction;
                if (target_p->heading_deg >= 360.0) target_p->heading_deg -= 360.0f;
                else if (target_p->heading_deg < 0) target_p->heading_deg += 360.0f;
            }

        }
        else
        {
            // keep heading_deg towards leash location
            target_p->heading_deg = target_navigation_leash.bearing_deg + course_correction;
            if (target_p->heading_deg >= 360.0) target_p->heading_deg -= 360.0f;
            else if (target_p->heading_deg < 0) target_p->heading_deg += 360.0f;


            // this calculations are unitless
            vector2_t gps_to_leash_diff;
            gps_to_leash_diff.x = target_leash_location.latitude - gnss_ptr->latitude;
            gps_to_leash_diff.y = target_leash_location.longitude - gnss_ptr->longitude;

            if (vector2_magnitude(&gps_to_leash_diff) < LEASH_LENGHT || is_leash_left_behind() == 1)
            {
                vector2_t leash_to_target_diff;

                leash_to_target_diff.x = target_p->latitude - target_leash_location.latitude;
                leash_to_target_diff.y = target_p->longitude - target_leash_location.longitude;

                if (vector2_magnitude(&leash_to_target_diff) > LEASH_LENGHT)
                {
                    leash_vector.x = target_p->latitude - target_leash_location.latitude;
                    leash_vector.y = target_p->longitude - target_leash_location.longitude;

                    norm_vector2(&leash_vector);

                    target_leash_location.latitude += leash_vector.x * LEASH_LENGHT;
                    target_leash_location.longitude += leash_vector.y * LEASH_LENGHT;
                }
                else
                {
                    is_leash_at_the_target = 1;

                    target_leash_location.latitude = target_p->latitude;
                    target_leash_location.longitude = target_p->longitude;
                    prev_target_navigation_distance_cm = 100000.0f;
                }

            }
        }



        float heading_diff = state_p->heading_deg - target_p->heading_deg;
        if (heading_diff < -180.0f) heading_diff += 360.0f;
        else if (heading_diff > 180.0f) heading_diff -= 360.0f;

        velocity_from_distance = target_navigation_wp.distance_cm / config_p->wp_dis_vel_gain;
        if (velocity_from_distance > config_p->max_horiz_vel) velocity_from_distance = config_p->max_horiz_vel;


        if (is_leash_at_the_target == 0)
        {
            if (fabs(heading_diff) < 90.0f) // 180 disables it
            {
                float vel_x = velocity_from_distance * cosf(heading_diff * DEG_TO_RAD);
                float vel_y = velocity_from_distance * sinf(-heading_diff * DEG_TO_RAD);

                target_p->velocity_x_ms += (vel_x - target_p->velocity_x_ms) * 0.2f;
                target_p->velocity_y_ms += (vel_y - target_p->velocity_y_ms) * 0.2f;
            }
            else
            {
                // slowly bring it down to zero
                target_p->velocity_x_ms -= target_p->velocity_x_ms * 0.2f;
                target_p->velocity_y_ms -= target_p->velocity_y_ms * 0.2f;
            }
        }
        else
        {
            if (velocity_from_distance < 1.5f) velocity_from_distance = 1.5;

            target_p->velocity_x_ms = velocity_from_distance * cosf(heading_diff * DEG_TO_RAD);
            target_p->velocity_y_ms = velocity_from_distance * sinf(-heading_diff * DEG_TO_RAD);
        }


        if (sqrtf(gnss_ptr->northVel_mms * gnss_ptr->northVel_mms + gnss_ptr->eastVel_mms * gnss_ptr->eastVel_mms) > 1000.0f)
        {
            float cog_wp_heading_diff = target_navigation_leash.bearing_deg - (gnss_ptr->headingOfMotion / 100000.0f);

            if (cog_wp_heading_diff < -180.0f) cog_wp_heading_diff += 360.0f;
            else if (cog_wp_heading_diff > 180.0f) cog_wp_heading_diff -= 360.0f;

            if (fabs(cog_wp_heading_diff) < 20.0f)
            {
                course_correction += (gps_heading_wp_heading_diff - calculated_heading_correction) * config_p->wp_hdg_cor_gain;
            }
                    
        }


        pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
        pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

        pid.posXPout = pid.errVel_x * config_p->pos_p;
        pid.posYPout = pid.errVel_y * config_p->pos_p;

        // output is saturated. dont wind up.
        if (fabs(target_p->pitch_deg) < POS_CTRL_MAX_PITCH_ROLL_DEG)
        {
            pid.posXIout += config_p->pos_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
            limit_symmetric(&pid.posXIout, POS_I_CTRL_MAX);
        }

        // output is saturated. dont wind up.
        if (fabs(target_p->roll_deg) < POS_CTRL_MAX_PITCH_ROLL_DEG)
        {
            pid.posYIout += config_p->pos_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
            limit_symmetric(&pid.posYIout, POS_I_CTRL_MAX);
        }

        pid.posXIout -= pid.posYIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);
        pid.posYIout += pid.posXIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);

        // smooth out requested pitch_deg from autopilot
        target_p->pitch_deg += ((pid.posXPout + pid.posXIout) - target_p->pitch_deg) * 0.2f;
        limit_symmetric(&target_p->pitch_deg, POS_CTRL_MAX_PITCH_ROLL_DEG);

        // smooth out requested roll_deg from autopilot
        target_p->roll_deg += ((pid.posYPout + pid.posYIout) - target_p->roll_deg) * 0.2f;
        limit_symmetric(&target_p->roll_deg, POS_CTRL_MAX_PITCH_ROLL_DEG);

        target_p->pitch_dps = calculate_target_pitch_degs_from_target_pitch_deg();
        target_p->roll_dps = calculate_target_roll_degs_from_target_roll_deg();
        target_p->yaw_dps = calculate_target_yaw_degs_from_target_heading_deg();

        ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
        // Throttle controls the altitude velocity
        if (radio_p->channel[RC_THROTTLE_CH] < 1550 && radio_p->channel[RC_THROTTLE_CH] > 1450) // Throttle stick is centered, velocity calculated from setpoint error
        {
            // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
            target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) * config_p->alt_vel_scale;
            limit_symmetric(&target_p->velocity_z_ms, config_p->max_vert_vel);
        }
        else // Throttle stick not centered, velocity calculated from stick input
        {
            // Calculate the desired altitude velocity from raw stick input
            target_p->velocity_z_ms = (apply_deadband(radio_p->channel[RC_THROTTLE_CH] - 1500.0f, 50) / 450.0f) * config_p->max_vert_vel;
            // we dont use altitude setpoint if the stick is not centered
            // but we want to set our setpoint when the stick is in middle
            // so that when we let go of the stick, craft stays at the altitude we let go
            target_p->altitude = state_p->altitude_m;
        }
    }

    // return 0 indicates navigation is in progress. craft did not reach the target location
    return 0;
}

static void outer_control_loop_rc(uint8_t land_flag)
{
    static uint8_t counter_set_point = 0;
    static uint8_t prev_land_flag = 0;

    counter_set_point++;
    if (counter_set_point >= 10) // 100Hz
    {
        counter_set_point = 0;

        // stick feed forward calculation
/*         deriv_rc_ch0 = (radio_p->channel[RC_ROLL_CH] - prev_rc_ch0);
        prev_rc_ch0 = radio_p->channel[RC_ROLL_CH];
        deriv_rc_ch1 = (radio_p->channel[RC_PITCH_CH] - prev_rc_ch1);
        prev_rc_ch1 = radio_p->channel[RC_PITCH_CH];
        deriv_rc_ch3 = (radio_p->channel[RC_YAW_CH] - prev_rc_ch3);
        prev_rc_ch3 = radio_p->channel[RC_YAW_CH];

        pid.pitch_ff_out += ((deriv_rc_ch1 * config_p->ff_gain) - pid.pitch_ff_out) * 0.4f;
        pid.roll_ff_out += ((deriv_rc_ch0 * config_p->ff_gain) - pid.roll_ff_out) * 0.4f;
        pid.yaw_ff_out += ((deriv_rc_ch3 * config_p->ff_gain) - pid.yaw_ff_out) * 0.4f; */

        // detect start of land command
        if (prev_land_flag == 0 && land_flag == 1)
        {
            // set heading_deg as takeoff heading_deg (can be overwritten by yaw stick)
            target_p->heading_deg = home.heading_deg;
            // reset landing detector
            landing_detector(1);
        }
        prev_land_flag = land_flag;

        if (flight_p->pos_hold_status == 1)
        {
            // pitch_deg / roll_deg stick centered
            if ((radio_p->channel[RC_PITCH_CH] > 1450 && radio_p->channel[RC_PITCH_CH] < 1550) && (radio_p->channel[RC_ROLL_CH] > 1450 && radio_p->channel[RC_ROLL_CH] < 1550))
            {
                // check if active gps hold can be used
                if (use_gps_hold == 1 && telemetry_p->is_gnss_sanity_check_ok == 1)
                {
                    // we hold the current location if hold location is selected
                    if (is_hold_location_set == 1)
                    {
                        get_distance_bearing(&target_navigation_wp, target_p->latitude, target_p->longitude, gnss_ptr->latitude, gnss_ptr->longitude);
                        telemetry_p->distance_m_2d = target_navigation_wp.distance_cm / 100.0f;
                        //telemetry_p->target_latitude = target_p->latitude;
                        //telemetry_p->target_longitude = target_p->longitude;

                        // we are outside hold threshold 
                        if (target_navigation_wp.distance_cm > config_p->wp_threshold_cm / 1.5f)
                        {
                            target_p->velocity_x_ms = (cosf(state_p->heading_deg * DEG_TO_RAD) * target_navigation_wp.distance_north_cm + cosf((state_p->heading_deg - 90.0f) * DEG_TO_RAD) * target_navigation_wp.distance_east_cm) / (config_p->wp_dis_vel_gain * 2.0f);
                            target_p->velocity_y_ms = (cosf(state_p->heading_deg * DEG_TO_RAD) * target_navigation_wp.distance_east_cm + cosf((state_p->heading_deg + 90.0f) * DEG_TO_RAD) * target_navigation_wp.distance_north_cm) / (config_p->wp_dis_vel_gain * 2.0f);

                            limit_symmetric(&target_p->velocity_x_ms, config_p->max_horiz_vel);
                            limit_symmetric(&target_p->velocity_y_ms, config_p->max_horiz_vel);
                        }
                        else
                        {
                            target_p->velocity_x_ms = 0;
                            target_p->velocity_y_ms = 0;
                        }
                    }
                    else
                    {
                        target_p->velocity_x_ms = 0;
                        target_p->velocity_y_ms = 0;
                        // we want to select a hold location if we are not moving fast
                        // if we choose a hold location while moving fast, we owershoot
                        // to prevent this first we have to slow down
                        if (fabs(state_p->vel_forward_ms) < 0.5 && fabs(state_p->vel_right_ms) < 0.5)
                        {
                            is_hold_location_set = 1;
                            target_p->latitude = gnss_ptr->latitude;
                            target_p->longitude = gnss_ptr->longitude;
                        }
                    }
                }
                else
                {   
                    // we dont have a good gps use passive hold
                    is_hold_location_set = 0;
                    target_p->velocity_x_ms = 0;
                    target_p->velocity_y_ms = 0;
                }
            }
            else    // pitch_deg / roll_deg stick not centered
            {
                is_hold_location_set = 0;
                // manual valocity control
                target_p->velocity_x_ms = ((radio_p->channel[RC_PITCH_CH] - 1500) / 500.0f) * config_p->max_horiz_vel;
                target_p->velocity_y_ms = ((radio_p->channel[RC_ROLL_CH] - 1500) / 500.0f) * config_p->max_horiz_vel;

                if (use_gps_hold == 1 && telemetry_p->is_gnss_sanity_check_ok == 1)
                {
                    target_p->latitude = gnss_ptr->latitude;
                    target_p->longitude = gnss_ptr->longitude;
                }
            }

            pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
            pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

            pid.posXPout = pid.errVel_x * config_p->pos_p;
            pid.posYPout = pid.errVel_y * config_p->pos_p;

            // output is saturated. dont wind up.
            if (fabs(target_p->pitch_deg) < POS_CTRL_MAX_PITCH_ROLL_DEG)
            {
                pid.posXIout += config_p->pos_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
                limit_symmetric(&pid.posXIout, POS_I_CTRL_MAX);
            }

            // output is saturated. dont wind up.
            if (fabs(target_p->roll_deg) < POS_CTRL_MAX_PITCH_ROLL_DEG)
            {
                pid.posYIout += config_p->pos_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
                limit_symmetric(&pid.posYIout, POS_I_CTRL_MAX);
            }

            pid.posXIout -= pid.posYIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);
            pid.posYIout += pid.posXIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);

            // smooth out requested pitch_deg from autopilot
            target_p->pitch_deg += ((pid.posXPout + pid.posXIout) - target_p->pitch_deg) * 0.2f;
            limit_symmetric(&target_p->pitch_deg, POS_CTRL_MAX_PITCH_ROLL_DEG);

            // smooth out requested roll_deg from autopilot
            target_p->roll_deg += ((pid.posYPout + pid.posYIout) - target_p->roll_deg) * 0.2f;
            limit_symmetric(&target_p->roll_deg, POS_CTRL_MAX_PITCH_ROLL_DEG);
        }
        else
        {
            target_p->pitch_deg = (apply_deadband(radio_p->channel[RC_PITCH_CH] - 1500, 20) / -480.0f) * config_p->max_pitch_angle;
            target_p->roll_deg = (apply_deadband(radio_p->channel[RC_ROLL_CH] - 1500, 20) / 480.0f) * config_p->max_roll_angle;
        }

        target_p->pitch_dps = calculate_target_pitch_degs_from_target_pitch_deg();
        target_p->roll_dps = calculate_target_roll_degs_from_target_roll_deg();

        /////////////////////   Yaw Rate Controller   ///////////////////////
        // Yaw stick controls yaw rate
        if (radio_p->channel[RC_YAW_CH] < 1550 && radio_p->channel[RC_YAW_CH] > 1450) // Yaw stick is centered
        {
            target_p->yaw_dps = calculate_target_yaw_degs_from_target_heading_deg();
        }
        else // Yaw stick is not centered
        {
            target_p->yaw_dps = ((radio_p->channel[RC_YAW_CH] - 1500.0f) / 500.0f) * config_p->max_yaw_rate;
            target_p->heading_deg = state_p->heading_deg;
        }
        // limit the yaw rate so it doesn't go crazy
        limit_symmetric(&target_p->yaw_dps, config_p->max_yaw_rate);

        if (flight_p->alt_hold_status == 1)
        {
            if (flight_p->takeoff_status == 1)
            {   
                // ramp up throttle to hover value slowly for takeoff
                pid.altIout += (config_p->hover_throttle - QUAD_IDLE_THROTTLE) / 100.0f;

                // when i value bigger than hover, set target altitude and let altitude controller take over
                if (pid.altIout >= config_p->hover_throttle)
                {
                    target_p->altitude = config_p->takeoff_alt;
                    flight_p->takeoff_status = 0;
                }
            }
            else if (land_flag == 0) // autoland off / Normal Z velocity control
            {
                ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
                // Throttle controls the altitude velocity
                if (radio_p->channel[RC_THROTTLE_CH] < 1550 && radio_p->channel[RC_THROTTLE_CH] > 1450) // Throttle stick is centered, velocity calculated from setpoint error
                {
                    // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
                    target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) * config_p->alt_vel_scale;
                    limit_symmetric(&target_p->velocity_z_ms, config_p->max_vert_vel);

                    if (flight_p->is_takeoff_done == 0 && (target_p->altitude - state_p->altitude_m) < 0.4f)
                    {
                        flight_p->is_takeoff_done = 1;
                    }
                }
                else // Throttle stick not centered, velocity calculated from stick input
                {
                    flight_p->is_takeoff_done = 1;
                    // 450 = 500 - deadband{50}
                    // Calculate the desired altitude velocity from raw stick input
                    target_p->velocity_z_ms = (apply_deadband(radio_p->channel[RC_THROTTLE_CH] - 1500.0f, 50) / 450.0f) * config_p->max_vert_vel;
                    // we dont use altitude setpoint if the stick is not centered
                    // but we want to set our setpoint when the stick is in middle
                    // so that when we let go of the stick, craft stays at the altitude we let go
                    target_p->altitude = state_p->altitude_m;
                }
            }
            else
            {
                // when landing is detected, disarm
                if (landing_detector(0) == 1)
                {
                    disarmed_by_landing = 1;
                    disarm();
                }
                // set target altitude to -1.0f to indicate landing
                target_p->altitude = -1.0f;
                // altitude > 0.3f we use normal descent
                if (state_p->altitude_m > 0.3f)
                {
                    target_p->velocity_z_ms += ((-state_p->altitude_m * config_p->alt_vel_scale) - target_p->velocity_z_ms) * 0.02f;
                }
                else
                {
                    // when we are close enough to ground we start killing the motors by slowly decreasing target velocity
                    target_p->velocity_z_ms -= 0.005f;
                }
                limit_symmetric(&target_p->velocity_z_ms, config_p->max_vert_vel);
            }

        }
        else
        {
            // (QUAD_MAX_TARGET_THROTTLE - QUAD_IDLE_THROTTLE) / 1000.0 = (800 - 300) / 1000.0 = 0.5
            target_p->throttle = (radio_p->channel[RC_THROTTLE_CH] - 1000.0f) * 0.5f + QUAD_IDLE_THROTTLE;

            if (target_p->throttle > QUAD_MAX_TARGET_THROTTLE) target_p->throttle = QUAD_MAX_TARGET_THROTTLE;
            else if (target_p->throttle < QUAD_IDLE_THROTTLE) target_p->throttle = QUAD_IDLE_THROTTLE;
        }
    }
}

static void inner_control_loop() // 1000Hz
{
    static float filt_target_pitch_dps;
    static float filt_target_roll_dps;
    static float filt_target_yaw_dps;
    
    // coordinate yaw turn when pitch_deg & roll_deg not zero
    float target_pitch_dps_corrected = sinf(state_p->roll_deg * DEG_TO_RAD) * target_p->yaw_dps + target_p->pitch_dps;
    float target_roll_dps_corrected = sinf(-state_p->pitch_deg * DEG_TO_RAD) * target_p->yaw_dps + target_p->roll_dps;
    float target_yaw_dps_corrected = fabs(cosf(state_p->roll_deg * DEG_TO_RAD)) * fabs(cosf(state_p->pitch_deg * DEG_TO_RAD)) * target_p->yaw_dps;

    // limit angular acceleration
    float pitch_requested_angular_accel = ((target_pitch_dps_corrected - filt_target_pitch_dps) * 0.1f) * 1000.0f;
    float roll_requested_angular_accel = ((target_roll_dps_corrected - filt_target_roll_dps) * 0.1f) * 1000.0f;
    float yaw_requested_angular_accel = ((target_yaw_dps_corrected - filt_target_yaw_dps) * 0.1f) * 1000.0f;

    if (pitch_requested_angular_accel > MAX_ANGULAR_ACCEL) filt_target_pitch_dps += MAX_ANGULAR_ACCEL * 0.001f;
    else if (pitch_requested_angular_accel < -MAX_ANGULAR_ACCEL) filt_target_pitch_dps -= MAX_ANGULAR_ACCEL * 0.001f;
    else filt_target_pitch_dps += (target_pitch_dps_corrected - filt_target_pitch_dps) * 0.1f;

    if (roll_requested_angular_accel > MAX_ANGULAR_ACCEL) filt_target_roll_dps += MAX_ANGULAR_ACCEL * 0.001f;
    else if (roll_requested_angular_accel < -MAX_ANGULAR_ACCEL) filt_target_roll_dps -= MAX_ANGULAR_ACCEL * 0.001f;
    else filt_target_roll_dps += (target_roll_dps_corrected - filt_target_roll_dps) * 0.1f;

    if (yaw_requested_angular_accel > MAX_ANGULAR_ACCEL) filt_target_yaw_dps += MAX_ANGULAR_ACCEL * 0.001f;
    else if (yaw_requested_angular_accel < -MAX_ANGULAR_ACCEL) filt_target_yaw_dps -= MAX_ANGULAR_ACCEL * 0.001f;
    else filt_target_yaw_dps += (target_yaw_dps_corrected - filt_target_yaw_dps) * 0.1f;


    // ↓↓↓↓↓↓↓↓↓↓   CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
    pid.errPitch = filt_target_pitch_dps - state_p->pitch_dps;
    pid.errRoll = filt_target_roll_dps - state_p->roll_dps;
    pid.errYaw = filt_target_yaw_dps - state_p->yaw_dps;
    // ↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   pitch_deg P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPout = pid.errPitch * config_p->pitch_p;
    // ↑↑↑↑↑↑↑↑↑↑   pitch_deg P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   roll_deg P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPout = pid.errRoll * config_p->roll_p;
    // ↑↑↑↑↑↑↑↑↑↑   roll_deg P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   YAW P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPout = pid.errYaw * config_p->yaw_p;
    biquad_lpf(&lpf_yaw_p_term, &pid.yawPout);
    // ↑↑↑↑↑↑↑↑↑↑   YAW P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    if (target_p->throttle > 320)
    {
        if (fabs(target_pitch_dps_corrected) < 25.0f)
        {
            // ↓↓↓↓↓↓↓↓↓↓  pitch_deg I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.pitchIout += config_p->pitch_i * 0.000625f * (pid.errPitch + pid.errPitchPrev); //  0.000625 = 0.5 * sampleTime
            limit_symmetric(&pid.pitchIout, MAX_I);
            // ↑↑↑↑↑↑↑↑↑↑   pitch_deg I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }

        if (fabs(target_roll_dps_corrected) < 25.0f)
        {
            // ↓↓↓↓↓↓↓↓↓↓   roll_deg I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.rollIout += config_p->roll_i * 0.000625f * (pid.errRoll + pid.errRollPrev);
            limit_symmetric(&pid.rollIout, MAX_I);
            // ↑↑↑↑↑↑↑↑↑↑   roll_deg I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }

        // ↓↓↓↓↓↓↓↓↓↓   YAW I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.yawIout += config_p->yaw_i * 0.000625f * (pid.errYaw + pid.errYawPrev);
        limit_symmetric(&pid.yawIout, MAX_I);
        // ↑↑↑↑↑↑↑↑↑↑   YAW I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
    }
    else
    {
        pid.pitchIout = 0.0f;
        pid.rollIout = 0.0f;
        pid.yawIout = 0.0f;
    }

    // ↓↓↓↓↓↓↓↓↓↓   pitch_deg D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchDout = config_p->pitch_d * (pid.errPitch - pid.errPitchPrev);
    biquad_lpf(&lpf_pitch_d_term, &pid.pitchDout);
    // ↑↑↑↑↑↑↑↑↑↑   pitch_deg D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   roll_deg D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollDout = config_p->roll_d * (pid.errRoll - pid.errRollPrev);
    biquad_lpf(&lpf_roll_d_term, &pid.rollDout);
    // ↑↑↑↑↑↑↑↑↑↑   roll_deg D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   pitch_deg PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPIDout = pid.pitchPout + pid.pitchIout + pid.pitchDout;
    limit_symmetric(&pid.pitchPIDout, MAX_PID);
    // ↑↑↑↑↑↑↑↑↑↑   pitch_deg PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   roll_deg PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPIDout = pid.rollPout + pid.rollIout + pid.rollDout;
    limit_symmetric(&pid.rollPIDout, MAX_PID);
    // ↑↑↑↑↑↑↑↑↑↑   roll_deg PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   YAW PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPIout = pid.yawPout + pid.yawIout;
    limit_symmetric(&pid.yawPIout, MAX_PID);
    // ↑↑↑↑↑↑↑↑↑↑   YAW PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.errPitchPrev = pid.errPitch;
    pid.errRollPrev = pid.errRoll;
    pid.errYawPrev = pid.errYaw;
    pid.pitchDegsPrev = state_p->pitch_dps;
    pid.rollDegsPrev = state_p->roll_dps;
    pid.yawDegsPrev = state_p->yaw_dps;
    // ↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    //=============================================================================//
    //                              Altitude Controller                            //
    //=============================================================================//
    static float accel_limited_target_z_velocity = 0;
    if (flight_p->alt_hold_status == 1)
    {
        static uint8_t counter = 0;
        counter++;
        if (counter >= 10) // 100 Hz
        {
            counter = 0;

            // calculate acceleration limited target velocity z
            float target_accel_z = (target_p->velocity_z_ms - accel_limited_target_z_velocity) * 100.0f;
            if (target_accel_z > MAX_VEL_Z_ACCEL) accel_limited_target_z_velocity += MAX_VEL_Z_ACCEL * 0.01f;
            else if (target_accel_z < -MAX_VEL_Z_ACCEL) accel_limited_target_z_velocity -= MAX_VEL_Z_ACCEL * 0.01f;
            else accel_limited_target_z_velocity += (target_p->velocity_z_ms - accel_limited_target_z_velocity);

            // ↓↓↓↓↓↓↓↓↓↓  CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
            pid.errVel_z = accel_limited_target_z_velocity - state_p->vel_up_ms;
            // ↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓  ALTITUDE P CALCULATION  ↓↓↓↓↓↓↓↓↓↓
            pid.altPout = pid.errVel_z * config_p->alt_p;
            // ↑↑↑↑↑↑↑↑↑↑  ALTITUDE P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓  ALTITUDE I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            if (flight_p->takeoff_status == 0)
            {
                pid.altIout += config_p->alt_i * 0.005f * (pid.errVel_z + pid.errVel_z_prev); //  0.005 = 0.5 * sampleTime
                if (pid.altIout > QUAD_MAX_TARGET_THROTTLE) pid.altIout = QUAD_MAX_TARGET_THROTTLE;
                else if (pid.altIout < QUAD_IDLE_THROTTLE) pid.altIout = QUAD_IDLE_THROTTLE;
            }
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE I CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   ALTITUDE D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.altDout = -config_p->alt_d * state_p->acc_up_ms2;
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   ALTITUDE PID OUT   ↓↓↓↓↓↓↓↓↓↓
            target_p->throttle = pid.altPout + pid.altIout + pid.altDout;
            if (target_p->throttle > QUAD_MAX_TARGET_THROTTLE) target_p->throttle = QUAD_MAX_TARGET_THROTTLE;
            else if (target_p->throttle < QUAD_IDLE_THROTTLE) target_p->throttle = QUAD_IDLE_THROTTLE;
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE PID OUT   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.errVel_z_prev = pid.errVel_z;
            pid.velocity_z_ms_prev = state_p->vel_up_ms;
            // ↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }
    }
    else
    {
        accel_limited_target_z_velocity = 0;
    }

    float comp_target_thr = target_p->throttle;
    // polinomial is specific to this drone
    // collect some data while hovering
    // substract initial throttle value from all data it should start from 0
    // remove begining and end of the data
    // gains = polyfit(batt_v, thr_zero, 2); this is the function for matlab (2 is for second order)
/*     if (telemetry_p->battery_voltage < 11.5f)
        comp_target_thr += telemetry_p->battery_voltage * -48.8911436f + 564.0f; */

    float cosAngAbs = cosf(fabs(state_p->pitch_deg) * DEG_TO_RAD) * cosf(fabs(state_p->roll_deg) * DEG_TO_RAD);
    if (cosAngAbs != 0)
        comp_target_thr += ((1.0f / cosAngAbs) - 1.0f) * comp_target_thr;


    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 1 (LEFT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr.m1 = comp_target_thr - pid.pitchPIDout + pid.rollPIDout + pid.yawPIout + pid.pitch_ff_out + pid.roll_ff_out + pid.yaw_ff_out;
    if (thr.m1 < MIN_THROTTLE) thr.m1 = MIN_THROTTLE;
    else if (thr.m1 > MAX_THROTTLE) thr.m1 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 1 (LEFT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 2 (LEFT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr.m2 = comp_target_thr + pid.pitchPIDout + pid.rollPIDout - pid.yawPIout - pid.pitch_ff_out + pid.roll_ff_out - pid.yaw_ff_out;
    if (thr.m2 < MIN_THROTTLE) thr.m2 = MIN_THROTTLE;
    else if (thr.m2 > MAX_THROTTLE) thr.m2 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 2 (LEFT TOP)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 3 (RIGHT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr.m3 = comp_target_thr - pid.pitchPIDout - pid.rollPIDout - pid.yawPIout + pid.pitch_ff_out - pid.roll_ff_out - pid.yaw_ff_out;
    if (thr.m3 < MIN_THROTTLE) thr.m3 = MIN_THROTTLE;
    else if (thr.m3 > MAX_THROTTLE) thr.m3 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 3 (RIGHT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 4 (RIGHT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr.m4 = comp_target_thr + pid.pitchPIDout - pid.rollPIDout + pid.yawPIout - pid.pitch_ff_out - pid.roll_ff_out + pid.yaw_ff_out;
    if (thr.m4 < MIN_THROTTLE) thr.m4 = MIN_THROTTLE;
    else if (thr.m4 > MAX_THROTTLE) thr.m4 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 4 (RIGHT TOP)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   OUTPUT TO THE MOTORS   ↓↓↓↓↓↓↓↓↓↓
    #if BENCH_MODE == 1
        set_throttle_quadcopter(0, 0, 0, 0);
    #else
        set_throttle_quadcopter(thr.m1, thr.m2, thr.m3, thr.m4);
    #endif
    //  ↑↑↑↑↑↑↑↑↑↑   OUTPUT TO THE MOTORS   ↑↑↑↑↑↑↑↑↑↑
}

static void arm()
{
    target_p->throttle = QUAD_IDLE_THROTTLE;
    pid.pitchDegsPrev = state_p->pitch_dps;
    pid.rollDegsPrev = state_p->roll_dps;
    pid.yawDegsPrev = state_p->yaw_dps;

    pid.velocity_x_ms_prev = state_p->vel_forward_ms;
    pid.velocity_y_ms_prev = state_p->vel_right_ms;
    pid.velocity_z_ms_prev = state_p->vel_up_ms;

    target_p->pitch_deg = 0;
    target_p->pitch_dps = 0;

    target_p->roll_deg = 0;
    target_p->roll_dps = 0;

    target_p->heading_deg = state_p->heading_deg;
    target_p->yaw_dps = 0;

    target_p->altitude = 0;
    target_p->velocity_x_ms = 0;
    target_p->velocity_y_ms = 0;
    target_p->velocity_z_ms = 0;

    pid.pitchIout = 0;
    pid.rollIout = 0;
    pid.yawIout = 0;
    pid.altIout = 0;
    pid.posXIout = 0;
    pid.posYIout = 0;

    pid.errPitchPrev = 0;
    pid.errRollPrev = 0;
    pid.errYawPrev = 0;

    if (telemetry_p->is_gnss_sanity_check_ok == 1)
    {
        home.latitude = gnss_ptr->latitude;
        home.longitude = gnss_ptr->longitude;
        home.altitude_mm = gnss_ptr->altitude_mm;
    }
    else
    {
        home.latitude = 0;
        home.longitude = 0;
        home.altitude_mm = 0;
    }

    telemetry_p->gps_latitude_origin = home.latitude;
    telemetry_p->gps_longitude_origin = home.longitude;
    telemetry_p->gps_altitude_origin = home.altitude_mm;

    home.heading_deg = state_p->heading_deg;

    // if altitude hold mode enabled before arming
    // then when armed, automatically takeoff pre determined altitude
    flight_p->takeoff_status = 0;
    if (flight_p->alt_hold_status == 1)
    {
        flight_p->takeoff_status = 1;
        pid.altIout = QUAD_IDLE_THROTTLE;
    }

    telemetry_p->arm_status = 1;
    flight_p->arm_status = 1;
}

uint8_t gnss_sanity_check()
{   
    // manually trigger gnss not safe for testing purposes
    //if (radio_p->ch8 <= 1300) return 0;
    // manually trigger gnss safe for testing purposes
    if (gnss_ptr->satCount > 7 && gnss_ptr->hdop < 300 && gnss_ptr->fix == 3) return 1;
    return 0;
}

static float calculate_target_yaw_degs_from_target_heading_deg()
{
    // Calculate yaw rate from setpoint error
    static float degs = 0.0f;
    degs = (target_p->heading_deg - state_p->heading_deg) * config_p->yaw_rate_scale;
    // This part ensures the craft turns from closest side to setpoint
    // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
    // If we dont do this craft will attempt to turn counter clockwise 265deg
    if (degs < -180.0f * config_p->yaw_rate_scale) degs += 360.0f * config_p->yaw_rate_scale;
    else if (degs > 180.0f * config_p->yaw_rate_scale) degs -= 360.0f * config_p->yaw_rate_scale;
    return degs;
}

static float calculate_target_pitch_degs_from_target_pitch_deg()
{
    /////////////////////   pitch_deg Rate Controller   ///////////////////////
    static float degs = 0.0f;
    degs = (target_p->pitch_deg - state_p->pitch_deg) * config_p->pitch_rate_scal;
    limit_symmetric(&degs, config_p->max_pitch_rate);
    return degs;
}

static float calculate_target_roll_degs_from_target_roll_deg()
{
    /////////////////////   roll_deg Rate Controller   ///////////////////////
    static float degs = 0.0f;
    degs = (target_p->roll_deg - state_p->roll_deg) * config_p->roll_rate_scal;
    limit_symmetric(&degs, config_p->max_roll_rate);
    return degs;
}

static uint8_t landing_detector(uint8_t need_reset)
{
    static uint16_t observed_throttle_value = 0;

    if (need_reset == 0)
    {   
        // lowpass throttle value to decide when to say landed
        observed_throttle_value += (target_p->throttle - observed_throttle_value) * 0.01f;

        if (state_p->altitude_m < 0.3f && observed_throttle_value < QUAD_IDLE_THROTTLE + 100.0f)
        {
            return 1;
        }
    }
    else
    {
        observed_throttle_value = config_p->hover_throttle;
    }
    return 0;
}

static void disarm()
{
    telemetry_p->arm_status = 0;
    flight_p->arm_status = 0;
    flight_p->is_takeoff_done = 0;

    set_throttle_quadcopter(0, 0, 0, 0);
}

static int16_t apply_deadband(int16_t input, uint16_t deadband)
{
    if (input > deadband || input < -deadband)
        return (input > 0) ? (input - deadband) : (input + deadband);
    else
        return 0;
}

static void get_distance_bearing(target_nav_t *tar_nav, int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
    // variable declerations
    static float lat1_rad = 0;
    static float lat2_rad = 0;
    static float lon1_rad = 0;
    static float lon2_rad = 0;
    static float dLat = 0;
    static float dLon = 0;
    static float sin_dlat = 0;
    static float sin_dlon = 0;
    static float cos_lat1 = 0;
    static float a = 0;

    // integer degree to float radian ((PI / 180) / 10000000)
    lat1_rad = lat1 * 1.745329252e-9f;
    lat2_rad = lat2 * 1.745329252e-9f;
    lon1_rad = lon1 * 1.745329252e-9f;
    lon2_rad = lon2 * 1.745329252e-9f;
    
    // calculate lat lon difference
    dLat = lat2_rad - lat1_rad;
    dLon = lon2_rad - lon1_rad;
    
    // calculate haversine distance 
    sin_dlat = sinf(dLat / 2.0f);
    sin_dlon = sinf(dLon / 2.0f);
    cos_lat1 = cosf(lat1_rad);
    
    a = sin_dlat * sin_dlat;
    tar_nav->distance_north_cm = atan2f(sqrtf(a), sqrtf(1.0f - a)) * EARTH_2_RADIUS_CM;

    if (dLat > 0)
        tar_nav->distance_north_cm = -tar_nav->distance_north_cm;
    
    a = cos_lat1 * cos_lat1 * sin_dlon * sin_dlon;
    tar_nav->distance_east_cm = atan2f(sqrtf(a), sqrtf(1.0f - a)) * EARTH_2_RADIUS_CM;
    
    if (dLon > 0)
        tar_nav->distance_east_cm = -tar_nav->distance_east_cm;
    
    tar_nav->distance_cm = sqrtf(tar_nav->distance_north_cm * tar_nav->distance_north_cm + tar_nav->distance_east_cm * tar_nav->distance_east_cm);
    tar_nav->bearing_deg = atan2f(tar_nav->distance_east_cm, tar_nav->distance_north_cm) * RAD_TO_DEG;
    
    if (tar_nav->bearing_deg < 0) tar_nav->bearing_deg += 360.0f;
}

static void navigation_start(int32_t latitude, int32_t longitude)
{
    // if waypoint counter is zero then find the vector that points from current location to first waypoint
    leash_vector.x = latitude - gnss_ptr->latitude;
    leash_vector.y = longitude - gnss_ptr->longitude;

    is_leash_at_the_target = 0;

    prev_target_navigation_distance_cm = 100000.0f;

    if (vector2_magnitude(&leash_vector) > LEASH_LENGHT)
    {
        // normalize the vector that points to next waypoint. leash will go towards this direction
        norm_vector2(&leash_vector);
        // set leash to current position + bit forward towards next location
        target_leash_location.latitude = gnss_ptr->latitude + (leash_vector.x * LEASH_LENGHT);
        target_leash_location.longitude = gnss_ptr->longitude + (leash_vector.y * LEASH_LENGHT);
    }
    else
    {
        // normalize the vector that points to next waypoint. leash will go towards this direction
        norm_vector2(&leash_vector);
        target_leash_location.latitude = latitude;
        target_leash_location.longitude = longitude;
    }
}

static uint8_t is_leash_left_behind()
{
    // variable declerations
    static int32_t vec_x = 0;
    static int32_t vec_y = 0;
    static int32_t vec_to_point_x = 0;
    static int32_t vec_to_point_y = 0;
    static int32_t dot_product = 0;

    // calculate vector from leash point to waypoint
    vec_x = target_p->latitude - target_leash_location.latitude;
    vec_y = target_p->longitude - target_leash_location.longitude;

    // check if leash left behind
    vec_to_point_x = gnss_ptr->latitude - target_leash_location.latitude;
    vec_to_point_y = gnss_ptr->longitude - target_leash_location.longitude;
    dot_product = vec_x * vec_to_point_x + vec_y * vec_to_point_y;

    if (dot_product > 100) return 1; // if dot_product > 0 leash is behind the craft but we put some margin
    return 0; // leash is not behind the craft
}

