#ifndef QUATERNION_H
#define QUATERNION_H

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f

typedef struct 
{
    float w, x, y, z;
} quat_t;

typedef struct
{
    float x, y, z;
} vector3_t;

typedef struct
{
    float x, y;
} vector2_t;

vector3_t cross_product(vector3_t *vec1, vector3_t *vec2);
void get_quat_deriv(quat_t *q_ptr, vector3_t *vect, quat_t *q_dot_ptr);
void norm_vector3(vector3_t *vector_ptr);
void norm_vector2(vector2_t *vector_ptr);
void norm_quat(quat_t *quat_ptr);
quat_t quat_conj(quat_t *q_ptr);
void quat_to_euler(quat_t *q_ptr, vector3_t *att);
void get_attitude_from_accel(vector3_t *vec, quat_t *q_result);
void get_heading_from_mag(vector3_t *vec, quat_t *q_result);
quat_t get_quat_product(quat_t *q1, quat_t *q2);
float angle_between_vectors(vector3_t *vec1, vector3_t *vec2);
float vector3_magnitude(vector3_t *vec);
float vector2_magnitude(vector2_t *vec);
float dot_product(vector3_t *vec1, vector3_t *vec2);
void get_quat_from_vector_measurements(vector3_t *vec_acc,vector3_t *vec_mag, quat_t *q_result);
void set_heading_quat(float pitch_deg, float roll_deg, vector3_t *vec_mag, quat_t *q_result);
void limit_symmetric(float *value, float limit);

#endif /*QUATERNION_H*/