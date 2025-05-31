#include "ecl_c_wrapper.h"
#include "../components/ecl/ecl.h"
#include "../components/ecl/EKF/ekf.h"
#include "../components/ecl/EKF/common.h"

// Create a C-compatible EKF object handle
extern "C" {

    Ekf ekf;

    void ekf_init() 
    {
        ekf.init(0);
    }


}