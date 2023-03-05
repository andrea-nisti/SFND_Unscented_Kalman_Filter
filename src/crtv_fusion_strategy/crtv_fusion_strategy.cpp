#include "crtv_fusion_strategy.h"

void CRTVFusionStrategy::InitFilter(Eigen::VectorXd init_state, Eigen::MatrixXd init_cov_matrix){
    Eigen::Ref<decltype(crtv_ukf_)::StateVector_t> init_state_ref{init_state};
    Eigen::Ref<decltype(crtv_ukf_)::StateCovMatrix_t> init_cov_matrix_ref{init_cov_matrix};

    crtv_ukf_.Init(init_state_ref, init_cov_matrix_ref);

}