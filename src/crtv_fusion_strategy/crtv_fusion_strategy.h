#ifndef SRC_CRTV_FUSION_STRATEGY_H
#define SRC_CRTV_FUSION_STRATEGY_H

#include "simpleukf/ukf/ukf.h"
#include "simpleukf/models/crtv_models.h"
#include <Eigen/Dense>

class CRTVFusionStrategy
{

  public:
    void InitFilter(Eigen::VectorXd init_state, Eigen::MatrixXd init_cov_matrix);

  private:
    simpleukf::ukf::UKF<simpleukf::models::CRTVModel<>> crtv_ukf_;
};

#endif  // SRC_CRTV_FUSION_STRATEGY_H
