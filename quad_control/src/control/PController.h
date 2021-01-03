#ifndef _P_CONTROLLER_
#define _P_CONTROLLER_

#include <Eigen/Dense>

#include "Controller.h"
#include "LP2Filter.h"

using namespace Eigen;

/*
 * This class represents a passivity-based controller, i.e., a hierarchical
 * controller embedded with an estimator of external wrench and unmodeled
 * dynamics.
 * This is implemented as a subclass of Controller (the hierarchical one).
 */
class PController : public Controller {

public:
    PController();
    ~PController(){}

private:
    bool _enableEstimator;          // If true, external wrench is estimated
    double _c0;                     // Estimator params
    Vector6d _Fe;                   // Estimated wrench

    Matrix3d _M;
    double _v;
    Matrix3d _Ko, _Do;              // Orientation gains
    Matrix3d _Kp, _Kd;              // Position gains

    Vector6d _q_prev;
    LP2Filter<Vector6d> _estFilter; // Low-pass filter for estimates


    /* Estimate external wrench based on the momentum of the system. */
    void _estimateWrench();

    /* Compute tau, i.e., the control torques. */
    void _computeTau();

    /* Compute mu_d. */
    void _computeMu();

    /* Controller core loop. */
    void _coreLoop();

};

#endif
