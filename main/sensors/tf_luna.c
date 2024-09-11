#include "sensors/tf_luna.h"
#include "typedefs.h"
#include "comminication/i2c.h"
#include "filters.h"
#include "math.h"

static uint8_t buffer[2];
static int16_t range;
static median_filter_t filter = {5, {0}, {0}};

void tf_luna_read_range(range_finder_t *range_ptr, states_t *state_ptr)
{
    i2c_read_bytes(I2C_NUM_0, SLAVE_ADDRESS, OUTX_L_G, buffer, 2);
    range = (int16_t)(buffer[0] | buffer[1] << 8);
    range = median_filter(&filter, (float)range);
    // roll can exceed +- 90 deg. tilt compnasation should not be negative. this is why we use fabs() on roll
    range_ptr->range_cm = (int16_t)((cosf(state_ptr->pitch_deg * DEG_TO_RAD) * fabsf(cosf(state_ptr->roll_deg * DEG_TO_RAD))) * (float)range);
}