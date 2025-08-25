#pragma once
#include "crsf_protocol.h"
#include "config.h"
#include "gyro.h"
#include "logging.h"

extern uint16_t midpoint[GYRO_MAX_CHANNELS];
extern bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS];

class Controller
{
public:
    virtual ~Controller() = default;

    void configure()
    {
        axisChannel[GYRO_AXIS_ROLL] = getChannel(MIX_DESTINATION_GYRO_ROLL);
        axisChannel[GYRO_AXIS_PITCH] = getChannel(MIX_DESTINATION_GYRO_PITCH);
        axisChannel[GYRO_AXIS_YAW] = getChannel(MIX_DESTINATION_GYRO_YAW);
        initialize();
    }

    virtual void update() = 0;

protected:
    int8_t axisChannel[3] = {-1, -1, -1};

    virtual void initialize() = 0;

    void setOutput(const gyro_axis_t axis, const float value) const
    {
        gyroCorrectionData[axis] = (value + get_command(axis)) * CRSF_CHANNEL_VALUE_MID;
    }

    void setOutputRaw(const gyro_axis_t axis, const float value) const
    {
        gyroCorrectionData[axis] = value * CRSF_CHANNEL_VALUE_MID;
    }

    static int8_t getChannel(const mix_destination_t destination)
    {
        for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
        {
            const rx_config_mix_t *mix = config.GetMix(mix_number);
            if (mix->val.active && mix->val.destination == destination)
            {
                return mix->val.source;
            }
        }
        return -1;
    }

    /**
     * Get a -1 to +1 float for the gyro input command
     */
    float get_command(const gyro_axis_t axis) const
    {
        const auto ch = axisChannel[axis];
        if (ch < 0) return 0.0f;
        const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;
        const uint16_t us = CRSF_to_US(ChannelData[ch]) - (mid - GYRO_US_MID);
        return us <= GYRO_US_MID
            ? float(us - GYRO_US_MID) / (GYRO_US_MID - GYRO_US_MIN)
            : float(us - GYRO_US_MID) / (GYRO_US_MAX - GYRO_US_MID);
    }

    float getChannelData(const gyro_axis_t axis) const
    {
        const auto ch = axisChannel[axis];
        if (ch < 0) return 0.0f;
        return CRSF_to_FLOAT((uint16_t) ChannelMixedData[ch]);
    }

    static void configure_pid_gains(PID * const pid, const rx_config_gyro_gains_t *gains, const float max, const float min)
    {
        DBGLN("Config gains: P %d I %d D %d G %d", gains->p, gains->i, gains->d, gains->gain);
        if (max == 0.0 && min == 0.0) {
            // No gyro correction on this axis
            pid->configure(0.0, 0.0, 0.0, 0.0, 0.0);
        } else {
            const float p = gains->gain * gains->p / 1000.0;
            const float i = gains->gain * gains->i / 1000.0;
            const float d = gains->gain * gains->d / 1000.0;
            DBGLN("PID gains: P %f I %f D %f", p, i, d);

            pid->configure(p, i, d, max, min);
        }
        pid->reset();
    }

    static void configure_pids(const float roll_limit, const float pitch_limit, const float yaw_limit)
    {
        const rx_config_gyro_gains_t *roll_gains = config.GetGyroGains(GYRO_AXIS_ROLL);
        const rx_config_gyro_gains_t *pitch_gains = config.GetGyroGains(GYRO_AXIS_PITCH);
        const rx_config_gyro_gains_t *yaw_gains = config.GetGyroGains(GYRO_AXIS_YAW);

        configure_pid_gains(&pid_roll, roll_gains, roll_limit, -1.0 * roll_limit);
        configure_pid_gains(&pid_pitch, pitch_gains, pitch_limit, -1.0 * pitch_limit);
        configure_pid_gains(&pid_yaw, yaw_gains, yaw_limit, -1.0 * yaw_limit);
    }

    static FusionQuaternion FusionQuaternionConjugate(const FusionQuaternion& q)
    {
        FusionQuaternion result;
        result.element.w = q.element.w;     // real part stays the same
        result.element.x = -q.element.x;    // imaginary parts are negated
        result.element.y = -q.element.y;
        result.element.z = -q.element.z;
        return result;
    }

    static FusionVector rotateVecByQuat(const FusionQuaternion q, const FusionVector v) {
        const FusionMatrix R = FusionQuaternionToMatrix(q);
        return (FusionVector){{
            R.array[0][0]*v.array[0] + R.array[0][1]*v.array[1] + R.array[0][2]*v.array[2],
            R.array[1][0]*v.array[0] + R.array[1][1]*v.array[1] + R.array[1][2]*v.array[2],
            R.array[2][0]*v.array[0] + R.array[2][1]*v.array[1] + R.array[2][2]*v.array[2],
        }};
    }

    // Body "down" (gravity) in BODY frame from the AHRS quaternion.
    // q_be: body->earth. We need earth->body, so take the conjugate and convert to a matrix.
    // The rotated [0,0,1] is simply the 3rd column of that matrix.
    static FusionVector quatBodyDown(const FusionQuaternion& q_be) {
        const FusionQuaternion q_eb = FusionQuaternionConjugate(q_be);
        const FusionMatrix R = FusionQuaternionToMatrix(q_eb);
        return (FusionVector){{ R.array[0][2], R.array[1][2], R.array[2][2] }};
    }

    // Roll/Pitch (radians) from the body "down" vector (no Euler sequence).
    // Sign matches the patterns we’ve been using (nose-up positive).
    static void quatToRollPitch(const FusionQuaternion& q_be, float& roll_rad, float& pitch_rad) {
        const FusionVector z_b = quatBodyDown(q_be);
        roll_rad  = atan2f(z_b.array[1], z_b.array[2]);
        pitch_rad = -atan2f(z_b.array[0], z_b.array[2]);
    }

    // EdgeSmoother: low-pass + slew only near/over the angle limit
    struct EdgeSmoother {
        // config (you can surface to your config class later)
        static constexpr float hyst_rad        = 1.0f * M_PI / 180.0f;  // 1° hysteresis
        static constexpr float smooth_margin   = 6.0f * M_PI / 180.0f;  // start smoothing within 6°
        static constexpr float tau_far         = 0.015f;                // ~15 ms far from limit
        static constexpr float tau_near        = 0.090f;                // ~90 ms near/over the limit
        static constexpr float slew_fs_per_s   = 3.0f;                  // full-scale steps per second

        // state
        float prev_cmd = 0.0f;

        void reset(float value = 0.0f) { prev_cmd = value; }

        float process(float angle_rad, float limit_rad, float raw_cmd, float dt) {
            // smoothing strength 0..1 (0 far inside, 1 at/over)
            float s;
            const float dist = limit_rad - fabsf(angle_rad);
            if (dist >= smooth_margin) s = 0.0f;
            else if (dist <= 0.0f)     s = 1.0f;
            else {
                const float x = 1.0f - (dist / smooth_margin);         // 0→1 approaching limit
                s = x*x*(3.0f - 2.0f*x);                                // smoothstep
            }

            // first-order low-pass (blend tau by s)
            const float tau   = tau_far * (1.0f - s) + tau_near * s;
            const float alpha = (tau <= 1e-6f) ? 1.0f : (dt / (tau + dt));
            const float filter = prev_cmd + alpha * (raw_cmd - prev_cmd);

            // slew-limit
            const float max_step = slew_fs_per_s * dt;
            const float delta = filter - prev_cmd;
            if      (delta >  max_step) prev_cmd += max_step;
            else if (delta < -max_step) prev_cmd -= max_step;
            else                        prev_cmd  = filter;

            return prev_cmd;
        }
    };
};