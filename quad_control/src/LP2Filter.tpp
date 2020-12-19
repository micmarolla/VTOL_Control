#include "LP2Filter.h"

template <typename T>
LP2Filter<T>::LP2Filter(){
    _f = 0;
    _d = _dd = 0;
    //_stepF = _stepD = _stepDD = 0;
    //_intOut1 = _intOut2 = 0;
}

template <typename T>
LP2Filter<T>::~LP2Filter(){
    delete _f;
    delete _d;
    delete _dd;
}

template <typename T>
void LP2Filter<T>::filter(T* signal, int length, double k1, double k2, T cond1, T cond2){
    delete _f;
    delete _d;
    delete _dd;

    _f = new T[length];
    _d = new T[length];
    _dd = new T[length];

    // Output of the integrators
    T intOut1 = cond1;
    T intOut2 = cond2;

    T sum1 = 0, sum2 = 0;

    for (int i=0; i < length; ++i){
        sum1 = signal[length] - intOut1;
        _d[i] = k1 * sum1;        
        intOut1 += _d[i];
        _f[i] = intOut1;
        sum2 = _d[i] - intOut2;
        _dd[i] = k2 * sum2;
        intOut2 += _dd[i];
    }
}


template <typename T>
void LP2Filter<T>::setInitCond(T cond1, T cond2){
    this->_intOut1 = cond1;
    this->_intOut2 = cond2;
}

template <typename T>
void LP2Filter<T>::setBandwidth(double k1, double k2){
    this->_k1 = k1;
    this->_k2 = k2;
}

template <typename T>
void LP2Filter<T>::initFilterStep(double k1, double k2, T cond1, T cond2){
    setBandwidth(k1, k2);
    setInitCond(cond1, cond2);
}

template <typename T>
void LP2Filter<T>::filterStep(T signal){
    T _sum1 = signal - _intOut1;
    _stepD = _k1 * _sum1;
    _intOut1 += _stepD;
    _stepF = _intOut1;
    T _sum2 = _stepD - _intOut2;
    _stepDD = _k2 * _sum2;
    _intOut2 += _stepDD;
}
