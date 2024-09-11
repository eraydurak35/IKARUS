#include "quaternion.h"
#include "math.h"

// get the cross product of two vector3
vector3_t cross_product(vector3_t *vec1, vector3_t *vec2)
{
    static vector3_t result = {0.0f, 0.0f, 0.0f};

    result.x = vec1->y * vec2->z - vec1->z * vec2->y;
    result.y = vec1->z * vec2->x - vec1->x * vec2->z;
    result.z = vec1->x * vec2->y - vec1->y * vec2->x;

    return result;
}

// get the dot product of two vector3
float dot_product(vector3_t *vec1, vector3_t *vec2)
{
    return vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
}

// returns magnitude of a vector3
float vector3_magnitude(vector3_t *vec)
{
    return sqrtf(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

// returns magnitude of a vector2
float vector2_magnitude(vector2_t *vec)
{
    return sqrtf(vec->x * vec->x + vec->y * vec->y);
}

// returns angle between two vectors in degrees
float angle_between_vectors(vector3_t *vec1, vector3_t *vec2)
{
    float dot = dot_product(vec1, vec2);
    float mag1 = vector3_magnitude(vec1);
    float mag2 = vector3_magnitude(vec2);
    return acosf(dot / (mag1 * mag2)) * RAD_TO_DEG;
}

// get quaternion derivative from gyro radians per second vector
// q_ptr -> pointer to current attitude
// vect -> pointer to gyro vector (radians per second)
// q_dot_ptr -> resulting quaternion derivative
void get_quat_deriv(quat_t *q_ptr, vector3_t *vect, quat_t *q_dot_ptr)
{
    q_dot_ptr->w = 0.5f * (-q_ptr->x * vect->x - q_ptr->y * vect->y - q_ptr->z * vect->z);
    q_dot_ptr->x = 0.5f * (q_ptr->w * vect->x + q_ptr->y * vect->z - q_ptr->z * vect->y);
    q_dot_ptr->y = 0.5f * (q_ptr->w * vect->y - q_ptr->x * vect->z + q_ptr->z * vect->x);
    q_dot_ptr->z = 0.5f * (q_ptr->w * vect->z + q_ptr->x * vect->y - q_ptr->y * vect->x);
}

// normalize vector3
void norm_vector3(vector3_t *vector_ptr)
{
    static float norm;
    norm = sqrtf(vector_ptr->x * vector_ptr->x + vector_ptr->y * vector_ptr->y + vector_ptr->z * vector_ptr->z);
    if (norm == 0) norm = 0.01f;
    vector_ptr->x /= norm;
    vector_ptr->y /= norm;
    vector_ptr->z /= norm;
}

// normalize vector2
void norm_vector2(vector2_t *vector_ptr)
{
    static float norm;
    norm = sqrtf(vector_ptr->x * vector_ptr->x + vector_ptr->y * vector_ptr->y);
    if (norm == 0) norm = 0.01f;
    vector_ptr->x /= norm;
    vector_ptr->y /= norm;
}

// normalize quaternion
void norm_quat(quat_t *quat_ptr)
{
    static float norm;
    norm = sqrtf(quat_ptr->w * quat_ptr->w + quat_ptr->x * quat_ptr->x + quat_ptr->y * quat_ptr->y + quat_ptr->z * quat_ptr->z);
    if (norm == 0) norm = 0.01f;
    quat_ptr->w /= norm;
    quat_ptr->x /= norm;
    quat_ptr->y /= norm;
    quat_ptr->z /= norm;
}

// returns conjugate of a quaternion
quat_t quat_conj(quat_t *q_ptr)
{
    static quat_t q_ret;

    q_ret.w = q_ptr->w;
    q_ret.x = -q_ptr->x;
    q_ret.y = -q_ptr->y;
    q_ret.z = -q_ptr->z;

    return q_ret;
}

// get euler representation of a quaternion
void quat_to_euler(quat_t *q_ptr, vector3_t *euler)
{
    static float a;

    a = 2.0f * q_ptr->x * q_ptr->z + 2.0f * q_ptr->w * q_ptr->y;

    if (a > 1.0f) a = 1.0f;
    else if (a < -1.0f) a = -1.0f;

    euler->x = -asinf(a);
    euler->y = atan2f((2.0f * q_ptr->y * q_ptr->z - 2.0f * q_ptr->w * q_ptr->x), (2.0f * q_ptr->w * q_ptr->w + 2.0f * q_ptr->z * q_ptr->z - 1.0f));
    euler->z = atan2f((2.0f * q_ptr->x * q_ptr->y - 2.0f * q_ptr->w * q_ptr->z), (2.0f * q_ptr->w * q_ptr->w + 2.0f * q_ptr->x * q_ptr->x - 1.0f));

    euler->x *= RAD_TO_DEG;
    euler->y *= RAD_TO_DEG;
    euler->z *= RAD_TO_DEG;

    euler->z += 180.0f;
}

// get quaternion attitude from only accelerometer vector
// Z axis should be negative
// vector should be normalize
void get_attitude_from_accel(vector3_t *vec, quat_t *q_result)
{
    if(vec->z >= 0.0f)
    {
        q_result->w = sqrtf((vec->z + 1.0f) / 2.0f);
        q_result->x = -vec->y / sqrtf(2.0f* (vec->z + 1.0f));
        q_result->y = vec->x / sqrtf(2.0f * (vec->z + 1.0f));
        q_result->z = 0.0f;
    }
    else
    {
        q_result->w = -vec->y / sqrtf(2.0f * (1.0f - vec->z));
        q_result->x = sqrtf((1.0f - vec->z) / 2.0f);
        q_result->y = 0.0f;
        q_result->z = vec->x / sqrtf(2.0f * (1.0f - vec->z));
    }

    norm_quat(q_result);
}

// get quaternion attitude from only magnetometer vector
// vector should be normalize
void get_heading_from_mag(vector3_t *vec, quat_t *q_result)
{
    float ro = vec->x * vec->x + vec->y * vec->y;

    if (vec->x >= 0.0f)
    {
        q_result->w = sqrtf(ro + vec->x * sqrtf(ro)) / sqrtf(2.0f * ro);
        q_result->x = 0.0f;
        q_result->y = 0.0f;
        q_result->z = vec->y / (sqrtf(2.0f) * sqrtf(ro + vec->x * sqrtf(ro)));
    }
    else
    {
        q_result->w = vec->y / (sqrtf(2.0f) * sqrtf(ro - vec->x * sqrtf(ro)));
        q_result->x = 0.0f;
        q_result->y = 0.0f;
        q_result->z = sqrtf(ro - vec->x * sqrtf(ro)) / sqrtf(2.0f * ro);
    }

    norm_quat(q_result);
}

// get quaternion product of two quaternion
quat_t get_quat_product(quat_t *q1, quat_t *q2)
{
    quat_t q_result;

    q_result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    q_result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    q_result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    q_result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

    norm_quat(&q_result);

    return q_result;
}


void get_quat_from_vector_measurements(vector3_t *vec_acc,vector3_t *vec_mag, quat_t *q_result)
{
    float pitch_radian = (asinf(vec_acc->y / sqrtf(vec_acc->x * vec_acc->x + vec_acc->y * vec_acc->y + vec_acc->z * vec_acc->z)));
    float roll_radian = -atan2f(vec_acc->x , vec_acc->z);

    float cos_pitch = cosf(pitch_radian);
    float cos_roll = cosf(-roll_radian);
    float sin_roll = sinf(-roll_radian);
    float sin_pitch = sinf(pitch_radian);

    float _X = -vec_mag->y * cos_pitch - vec_mag->x * sin_roll * sin_pitch + vec_mag->z * cos_roll * sin_pitch;
    float _Y = -vec_mag->x * cos_roll - vec_mag->z * sin_roll;
    float heading_radian = atan2f(_Y, _X) + M_PI;

    q_result->w = cosf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f) + sinf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f);
    q_result->x = sinf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f) - cosf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f);
    q_result->y = cosf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f) + sinf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f);
    q_result->z = cosf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f) - sinf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f);

    norm_quat(q_result);
}

void set_heading_quat(float pitch_deg, float roll_deg, vector3_t *vec_mag, quat_t *q_result)
{
    float pitch_radian = pitch_deg * DEG_TO_RAD;
    float roll_radian = roll_deg * DEG_TO_RAD;

    float cos_pitch = cosf(pitch_radian);
    float cos_roll = cosf(-roll_radian);
    float sin_roll = sinf(-roll_radian);
    float sin_pitch = sinf(pitch_radian);

    float _X = -vec_mag->y * cos_pitch - vec_mag->x * sin_roll * sin_pitch + vec_mag->z * cos_roll * sin_pitch;
    float _Y = -vec_mag->x * cos_roll - vec_mag->z * sin_roll;
    float heading_radian = atan2f(_Y, _X) + M_PI;

    q_result->w = cosf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f) + sinf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f);
    q_result->x = sinf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f) - cosf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f);
    q_result->y = cosf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f) + sinf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f);
    q_result->z = cosf(roll_radian / 2.0f) * cosf(pitch_radian / 2.0f) * sinf(heading_radian / 2.0f) - sinf(roll_radian / 2.0f) * sinf(pitch_radian / 2.0f) * cosf(heading_radian / 2.0f);

    norm_quat(q_result);
}

void limit_symmetric(float *value, float limit)
{
    if (*value > limit) *value = limit;
    else if (*value < -limit) *value = -limit;
}
