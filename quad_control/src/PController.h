#ifndef _P_CONTROLLER_
#define _P_CONTROLLER_

#include <Eigen/Dense>
#include "Controller.h"

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
    bool _enableEstimator;
    double _c0;                     // Estimator params
    Matrix<double,6,1> _Fe;         // Estimated wrench
    Matrix3d _M;
    double _v;
    Matrix3d _Ko, _Do;
    Matrix3d _Kp, _Kd;
    Matrix<double,6,1> _q_prev;
    LP2Filter<Matrix<double,6,1>> _estFilter;


    /*
     * Estimate external wrench based on the momentum of the system.
     */
    void _estimateWrench();

    void _computeTau();

    void _computeMu();

    void _coreLoop();

};

#endif
