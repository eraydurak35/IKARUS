#include "state_estimator.h"
#include "bmp390.h"
#include "math.h"
#include "typedefs.h"
#include "quaternion.h"
#include "esp_timer.h"
#include "setup.h"
// An Extended Complementary Filter (ECF) for Full-Body MARG Orientation Estimation
// DOI:10.1109/TMECH.2020.2992296

static quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
static quat_t q_dot = {0.0f, 0.0f, 0.0f, 0.0f};
static vector_t gyr_vec = {0.0f, 0.0f, 0.0f};
static vector_t acc_vec = {0.0f, 0.0f, 1.0f};
static vector_t mag_vec = {1.0f, 0.0f, 0.0f};
static vector_t err = {0.0f, 0.0f, 0.0f};
static vector_t local_vr_a = {0.0f, 0.0f, 0.0f};
#if SETUP_MAGNETO_TYPE != MAG_NONE
static vector_t local_vr_m = {0.0f, 0.0f, 0.0f};
#endif
static states_t *state_ptr = NULL;
static imu_t *imu_ptr = NULL;
static magnetometer_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static config_t *config_ptr = NULL;
static flight_t *flight_ptr = NULL;
static uint8_t movement_state = 0;
// Kalman filtre parametreleri
typedef struct {
    float x;   // Durum (yükseklik)
    float v;   // Hız
    float P[2][2];  // Durum kovaryans matrisi
    float Q[2][2];  // Süreç kovaryans matrisi
    float R;        // Ölçüm kovaryans matrisi (barometrik sensör)
} kalman_t;

kalman_t kf;

static void get_attitude_heading();
/* static float deadband(float value, const float threshold); */
static void kalman_init(kalman_t *kf, float initial_height, float initial_velocity, float process_noise, float measurement_noise);
static void kalman_predict(kalman_t *kf, float measured_acceleration, float dt);
static void kalman_update(kalman_t *kf, float measured_height);
static uint8_t is_movement_detected();

void ahrs_init(config_t *cfg, states_t *sta, imu_t *icm, magnetometer_t *hmc, bmp390_t *baro, flight_t *flt)
{
    config_ptr = cfg;
    state_ptr = sta;
    imu_ptr = icm;
    mag_ptr = hmc;
    baro_ptr = baro;
    flight_ptr = flt;

    acc_vec.x = imu_ptr->accel_ms2[Y];
    acc_vec.y = imu_ptr->accel_ms2[X];
    acc_vec.z = imu_ptr->accel_ms2[Z];

#if SETUP_MAGNETO_TYPE != MAG_NONE

    mag_vec.x = mag_ptr->axis[X];
    mag_vec.y = mag_ptr->axis[Y];
    mag_vec.z = mag_ptr->axis[Z];

#endif

    // Başlangıç anındaki quaternion duruşu ivme ve manyetik vektörlerden hesapla
    get_quat_from_vector_measurements(&acc_vec, &mag_vec, &q);

    kalman_init(&kf, 0, 0, 0.001, 10.0);
}


void ahrs_predict()
{
    static int64_t t = 0;
    float dt = ((esp_timer_get_time() - t)) / 1000000.0f;
    t = esp_timer_get_time();   

    movement_state = is_movement_detected();

    gyr_vec.x = imu_ptr->gyro_dps[Y] * DEG_TO_RAD;
    gyr_vec.y = imu_ptr->gyro_dps[X] * DEG_TO_RAD;
    gyr_vec.z = -imu_ptr->gyro_dps[Z] * DEG_TO_RAD;

    if (flight_ptr->arm_status == 0 && movement_state == 0)
    {
        err.x *= 2.0f;
        err.y *= 2.0f;
        err.z *= 2.0f;
    }

    gyr_vec.x -= (config_ptr->ahrs_filt_beta * err.x);
    gyr_vec.y -= (config_ptr->ahrs_filt_beta * err.y);
    gyr_vec.z -= (config_ptr->ahrs_filt_beta * err.z);

    get_quat_deriv(&q, &gyr_vec, &q_dot);

    q.w += q_dot.w * dt;
    q.x += q_dot.x * dt;
    q.y += q_dot.y * dt;
    q.z += q_dot.z * dt;

    norm_quat(&q);
}

void ahrs_correct()
{
    static vector_t err_acc = {0.0f, 0.0f, 0.0f};


    acc_vec.x = imu_ptr->accel_ms2[Y];
    acc_vec.y = imu_ptr->accel_ms2[X];
    acc_vec.z = -imu_ptr->accel_ms2[Z];

    mag_vec.x = mag_ptr->axis[X];
    mag_vec.y = -mag_ptr->axis[Y];
    mag_vec.z = mag_ptr->axis[Z];

    norm_vector(&acc_vec);
    norm_vector(&mag_vec);

    local_vr_a.x = 2.0f * ((q.x * q.z) - (q.w * q.y));
    local_vr_a.y = 2.0f * ((q.w * q.x) + (q.y * q.z));
    local_vr_a.z = 2.0f * ((q.w * q.w) + (q.z * q.z)) - 1.0f;

    err_acc = cross_product(&acc_vec, &local_vr_a);

#if SETUP_MAGNETO_TYPE != MAG_NONE

    local_vr_m.x = 2.0f * ((q.x * q.y) + (q.w * q.z));
    local_vr_m.y = 2.0f * ((q.w * q.w) + (q.y * q.y)) - 1.0f;
    local_vr_m.z = 2.0f * ((q.y * q.z) - (q.w * q.x));

    static vector_t err_mag = {0.0f, 0.0f, 0.0f};
    static vector_t temp = {0.0f, 0.0f, 0.0f};

    temp = cross_product(&acc_vec, &mag_vec);
    err_mag = cross_product(&temp, &local_vr_m);

    err.x = err_acc.x + err_mag.x;
    err.y = err_acc.y + err_mag.y;
    err.z = err_acc.z + err_mag.z;

    // if not armed, transitioning from movement to no movement
    // set heading to mag heading
    static uint8_t prev_movement_state = 0;
    if (flight_ptr->arm_status == 0 && (movement_state == 0 && prev_movement_state == 1))
    {
        mag_vec.x = mag_ptr->axis[X];
        mag_vec.y = mag_ptr->axis[Y];
        mag_vec.z = -mag_ptr->axis[Z];
        set_heading_quat(state_ptr->pitch_deg, state_ptr->roll_deg, &mag_vec, &q);
    }
    prev_movement_state = movement_state;

    // ground can distort magnetic heading. we need to reset magnetic heading when we are away from ground some distance
    if (flight_ptr->is_in_flight_mag_allign_done == 0 && flight_ptr->arm_status == 1 && state_ptr->altitude_m > IN_FLT_MAG_ALLN_ALT)
    {
        mag_vec.x = mag_ptr->axis[X];
        mag_vec.y = mag_ptr->axis[Y];
        mag_vec.z = -mag_ptr->axis[Z];
        set_heading_quat(state_ptr->pitch_deg, state_ptr->roll_deg, &mag_vec, &q);
        flight_ptr->is_in_flight_mag_allign_done = 1;
    }

    imu_ptr->gyro_bias_dps[Y] += err.x * config_ptr->ahrs_filt_zeta;
    imu_ptr->gyro_bias_dps[X] += err.y * config_ptr->ahrs_filt_zeta;
    imu_ptr->gyro_bias_dps[Z] -= err.z * config_ptr->ahrs_filt_zeta;

#else

    err.x = err_acc.x;
    err.y = err_acc.y;
    err.z = err_acc.z;

    imu_ptr->gyro_bias_dps[Y] += err.x * config_ptr->ahrs_filt_zeta;
    imu_ptr->gyro_bias_dps[X] += err.y * config_ptr->ahrs_filt_zeta;

#endif

    get_attitude_heading();
}

static void get_attitude_heading()
{
    static quat_t q_conj;
    static vector_t euler;

    q_conj = quat_conj(&q);
    quat_to_euler(&q_conj, &euler);

    state_ptr->pitch_deg = euler.x;
    state_ptr->roll_deg = euler.y;
    state_ptr->heading_deg = euler.z - config_ptr->mag_declin_deg;

    if (state_ptr->heading_deg < 0) state_ptr->heading_deg += 360.0f;
    else if (state_ptr->heading_deg > 360.0f) state_ptr->heading_deg -= 360.0f;

    state_ptr->pitch_dps = imu_ptr->gyro_dps[X];
    state_ptr->roll_dps = imu_ptr->gyro_dps[Y];
    state_ptr->yaw_dps = -imu_ptr->gyro_dps[Z];
}



// Heading (yaw) açısndan bağımsız olarak ivme ölçümlerini body frame'den earth frame'e geçir
void earth_frame_acceleration()
{
    static float rot_matrix[3][3] = {0};
    static float pitch_radians = 0.0f;
    static float roll_radians = 0.0f;
    static float cosx = 0.0f;
    static float sinx = 0.0f;
    static float cosy = 0.0f;
    static float siny = 0.0;

    pitch_radians = state_ptr->pitch_deg * DEG_TO_RAD;
    roll_radians = state_ptr->roll_deg * DEG_TO_RAD;

    cosx = cosf(roll_radians);
    sinx = sinf(roll_radians);
    cosy = cosf(pitch_radians);
    siny = sinf(pitch_radians);

    rot_matrix[0][0] = cosy;
    rot_matrix[0][1] = 0.0f;
    rot_matrix[0][2] = siny;
    rot_matrix[1][0] = sinx * siny;
    rot_matrix[1][1] = cosx;
    rot_matrix[1][2] = -sinx * cosy;
    rot_matrix[2][0] = -(cosx * siny);
    rot_matrix[2][1] = sinx;
    rot_matrix[2][2] = cosy * cosx;

    state_ptr->acc_forward_ms2 = imu_ptr->accel_ms2[Y] * rot_matrix[0][0] + imu_ptr->accel_ms2[X] * rot_matrix[1][0] + imu_ptr->accel_ms2[Z] * rot_matrix[2][0];
    state_ptr->acc_right_ms2 = imu_ptr->accel_ms2[Y] * rot_matrix[0][1] + imu_ptr->accel_ms2[X] * rot_matrix[1][1] + imu_ptr->accel_ms2[Z] * rot_matrix[2][1];
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * rot_matrix[0][2] + imu_ptr->accel_ms2[X] * rot_matrix[1][2] + imu_ptr->accel_ms2[Z] * rot_matrix[2][2]) - 9.806f;


/*  static float rot_matrix[3][3] = {0};

    rot_matrix[0][0] = (q.w * q.w) + (q.x * q.x) - (q.y * q.y) - (q.z * q.z);
    rot_matrix[0][1] = 2.0f * (q.x * q.y - q.w * q.z);
    rot_matrix[0][2] = 2.0f * (q.x * q.z + q.w * q.y);

    rot_matrix[1][0] = 2.0f * (q.x * q.y + q.w * q.z);
    rot_matrix[1][1] = (q.w * q.w) - (q.x * q.x) + (q.y * q.y) - (q.z * q.z);
    rot_matrix[1][2] = 2.0f * (q.y * q.z - q.w * q.x);

    rot_matrix[2][0] = 2.0f * (q.x * q.z - q.w * q.y);
    rot_matrix[2][1] = 2.0f * (q.w * q.x + q.y * q.z);
    rot_matrix[2][2] = (q.w * q.w) - (q.x * q.x) - (q.y * q.y) + (q.z * q.z);

    state_ptr->acc_right_ms2 = -(imu_ptr->accel_ms2[Y] * -rot_matrix[0][0] + imu_ptr->accel_ms2[X] * -rot_matrix[0][1] + imu_ptr->accel_ms2[Z] * rot_matrix[0][2]);
    state_ptr->acc_forward_ms2 = (imu_ptr->accel_ms2[Y] * -rot_matrix[1][0] + imu_ptr->accel_ms2[X] * -rot_matrix[1][1] + imu_ptr->accel_ms2[Z] * rot_matrix[1][2]);
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * -rot_matrix[2][0] + imu_ptr->accel_ms2[X] * -rot_matrix[2][1] + imu_ptr->accel_ms2[Z] * rot_matrix[2][2]) - 9.806f; */
}


void altitude_predict()
{
    static int64_t t = 0; 
    float dt = ((esp_timer_get_time() - t)) / 1000000.0f;
    t = esp_timer_get_time();

    kalman_predict(&kf, state_ptr->acc_up_ms2, dt);
}

void altitude_correct()
{
    kalman_update(&kf, baro_ptr->altitude_m);
}

/* static float deadband(float value, const float threshold)
{
    if (fabsf(value) < threshold) value = 0;
    else if (value > 0) value -= threshold;
    else if (value < 0) value += threshold;
    return value;
} */


// Kalman filtresi başlatma fonksiyonu
static void kalman_init(kalman_t *kf, float initial_height, float initial_velocity, float process_noise, float measurement_noise) 
{
    kf->x = initial_height;
    kf->v = initial_velocity;
    kf->P[0][0] = 1;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 1;
    kf->Q[0][0] = process_noise;
    kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;
    kf->Q[1][1] = process_noise;
    kf->R = measurement_noise;
}

// Predict adımı
static void kalman_predict(kalman_t *kf, float measured_acceleration, float dt) 
{
    // Ön tahmin
    kf->x = kf->x + kf->v * dt + 0.5f * measured_acceleration * dt * dt;
    kf->v = kf->v + measured_acceleration * dt;

    state_ptr->altitude_m = kf->x;
    state_ptr->vel_up_ms = kf->v;

    // Kovaryans matrisi güncellemesi
    kf->P[0][0] += dt * (2.0f * kf->P[0][1] + dt * kf->P[1][1]) + kf->Q[0][0];
    kf->P[0][1] += dt * kf->P[1][1];
    kf->P[1][0] += dt * kf->P[1][1];
    kf->P[1][1] += kf->Q[1][1];
}

// Update adımı
static void kalman_update(kalman_t *kf, float measured_height) 
{
    // Ölçüm yenilemesi
    float y = measured_height - kf->x; // Ölçüm yeniliği
    float S = kf->P[0][0] + kf->R; // Yenilik kovaryansı
    float K[2]; // Kalman kazancı
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Durum vektörü güncellemesi
    kf->x += K[0] * y;
    kf->v += K[1] * y;

    //printf("%.2f,%.2f\n", kf->x, baro_ptr->altitude_m);
    //printf("%.2f,%.2f\n", kf->v, baro_ptr->velocity_ms);

    //printf("%.4f,%.4f\n", K[0], K[1]);

    // Kovaryans matrisi güncellemesi
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
}


static uint8_t is_movement_detected()
{
    static float gyro_vector = 0;
    static float filt_vector = 0;
    gyro_vector = imu_ptr->gyro_dps[X] * imu_ptr->gyro_dps[X] + imu_ptr->gyro_dps[Y] * imu_ptr->gyro_dps[Y] + imu_ptr->gyro_dps[Z] * imu_ptr->gyro_dps[Z];
    if (gyro_vector > 2000.0f) gyro_vector = 2000.0f;
    filt_vector += (gyro_vector - filt_vector) * 0.001f;
    return (filt_vector > GYRO_MOVEMENT_DETECT_THRESHOLD) ? 1 : 0;;
}