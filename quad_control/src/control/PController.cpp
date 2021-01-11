#include "PController.h"
#include <ros/console.h>

using namespace Eigen;

PController::PController() : Controller(){
    _c0 = _nh.param<double>("c0", 1000.0);
    _v  = _nh.param<double>("v",  100.0);
    _enableEstimator = _nh.param<bool>("estimator",  true);

    double kp = _nh.param<double>("kp",  100.0);
    _Kp = kp * Matrix3d::Identity();
    double kd = _nh.param<double>("kd",  100.0);
    _Kd = kd * Matrix3d::Identity();

    double ko = _nh.param<double>("ko",  100.0);
    _Ko = ko * Matrix3d::Identity();
    _Do = _Ko / _v;

    double band = _nh.param<double>("filterBand", 10.0);
    _estFilter.initFilterStep(0.001, band, 0, Vector6d::Zero(), Vector6d::Zero());

    _q_prev = Vector6d::Zero();
    _Fe << Vector6d::Zero();
}


void PController::_estimateWrench(){
    Vector4d u; u << _uT, _tau;
    Vector6d sigma_d; sigma_d << _p_d, _eta_d;
    Matrix<double,6,6> M_sigma; M_sigma << _m*Matrix3d::Identity(), Matrix3d::Zero(),
                                            Matrix3d::Zero(),       _M;
    Vector6d q = M_sigma * sigma_d;
    Matrix<double,6,6> C_sigma; C_sigma << Matrix3d::Zero(),    Matrix3d::Zero(),
                                            Matrix3d::Zero(),   _C;
    Matrix<double,6,4> Delta; Delta << -_Rb*Vector3d(0,0,1),    Matrix3d::Zero(),
                                        Vector3d::Zero(),       _QT;
    Vector6d G_sigma; G_sigma << 0, 0, -_m*GRAVITY, 0, 0, 0;

    // Estimation
    Vector6d X = C_sigma.transpose() * sigma_d + Delta*u - G_sigma;
    Vector6d tempFe = _Fe + _c0 * (q-_q_prev) - (_c0*_Fe + _c0*X)/_rate;

    // Filter
    this->_estFilter.filterSteps(tempFe, 10);
    _Fe << _estFilter.lastFirst();

    _q_prev = q;
}


void PController::_computeMu(){
    Vector3d mud = Vector3d(_da.linear.x, _da.linear.y, _da.linear.z) -
        (_Kp * _e_p.head<3>() + _Kd * _e_p.tail<3>() +
         _Kpi * _epInt) / _m;

    _mud = mud - _Fe.head<3>() / _m;
}


void PController::_computeTau(){
    Vector3d e_eta = _e_eta.head<3>();
    Vector3d eta_r_d = _deta_d - _v * e_eta;
    Vector3d eta_r_dd = _deta_dd - _v * eta_r_d;
    Vector3d v_eta = _e_eta.tail<3>() + _v * e_eta;

    _tau = _Q_inv.transpose() *
        (_M*eta_r_dd + _C*eta_r_d - _Fe.tail<3>() - _Do*v_eta - _Ko*e_eta);
}


void PController::_coreLoop(){
    _M = _QT * _Ib * _Q;
    if (_enableEstimator)
        _estimateWrench();
    _outerLoop();
    _innerLoop();
}
