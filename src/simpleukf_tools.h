#ifndef SRC_SIMPLEUKF_TOOLS_H
#define SRC_SIMPLEUKF_TOOLS_H

#include "simpleukf/models/ctrv_models.h"
#include "simpleukf/ukf/linear_update_strategy.h"
#include "simpleukf/ukf/unscented_update_strategy.h"
#include "simpleukf/ukf/ukf.h"

using namespace simpleukf::models;

struct RadarNoiseConstants
{
    static constexpr double std_radr = 0.3;
    static constexpr double std_radphi = 0.03;
    static constexpr double std_radrd = 0.3;
};

struct LidarNoiseConstants
{
    // Laser measurement noise standard deviation position1 in m
    static constexpr double std_laspx = 0.15;

    // Laser measurement noise standard deviation position2 in m
    static constexpr double std_laspy = 0.15;
};

struct ProcessNoiseConstants
{
    static constexpr double nu_a = 3;
    static constexpr double nu_psi_dd = 0.6;
};

using CTRVProcessModel = CTRVModel<ProcessNoiseConstants>;
using RadarMeasurementModel = RadarModel<RadarNoiseConstants>;
using LidarMeasurementModel = LidarModel<LidarNoiseConstants>;

using CTRVFusionFilter = simpleukf::ukf::UKF<CTRVProcessModel>;

#endif // SRC_SIMPLEUKF_TOOLS_H
