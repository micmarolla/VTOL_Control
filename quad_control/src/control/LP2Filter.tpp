#include "LP2Filter.h"

template <typename T>
LP2Filter<T>::LP2Filter(){
    _f = _d = _dd = 0;
    _first = true;
}

template <typename T>
LP2Filter<T>::~LP2Filter(){
    delete _f;
    delete _d;
    delete _dd;
}

template <typename T>
void LP2Filter<T>::filter(T* signal, unsigned int length, double sampleTime,
        double k1, double k2, T cond1, T cond2, int steps){
    // Clear old signals
    delete _f;
    delete _d;
    delete _dd;

    if(length == 0)
        return;

    _f = new T[length];
    _d = new T[length];
    _dd = new T[length];

    // Initial conditions and parameters
    initFilterStep(sampleTime, k1, k2, cond1, cond2, steps);

    // Filter
    for(int i=0; i < length; ++i){
        this->filterStep(signal[i]);
        _f[i] = _stepF;
        _d[i] = _stepD;
        _dd[i] = _stepDD;
    }
}


template <typename T>
void LP2Filter<T>::setSampleTime(double t){
    _t = t;
}

template <typename T>
void LP2Filter<T>::setInitCond(T cond1, T cond2){
    _init1 = cond1;
    _init2 = cond2;
}

template <typename T>
void LP2Filter<T>::setBandwidth(double k1, double k2){
    this->_k1 = k1;
    this->_k2 = k2;
}

template <typename T>
void LP2Filter<T>::reset(){
    _first = true;
}


template <typename T>
void LP2Filter<T>::initFilterStep(double sampleTime, double k1, double k2,
        T cond1, T cond2, int steps){
    setSampleTime(sampleTime);
    setBandwidth(k1, k2);
    setInitCond(cond1, cond2);
    _steps = steps;
    reset();
}

template <typename T>
void LP2Filter<T>::filterStep(T signal){
    if(_first){                     // First simulation step
        _stepF = _init1;
        _stepD = _k1 * (signal - _stepF);
        _x1 = _stepF + _t * _stepD;

        _intOut2 = _init2;
        _stepDD = _k2 * (_stepD - _intOut2);
        _x2 = _intOut2 + _t * _stepDD;

        _first = false;
    }
    else{                           // Normal simulation step
        _stepF = _x1;
        _stepD = _k1 * (signal - _stepF);
        _x1 = _x1 + _t * _stepD;

        _intOut2 = _x2;
        _stepDD = _k2 * (_stepD - _intOut2);
        _x2 = _x2 + _t * _stepDD;
    }
}

template <typename T>
void LP2Filter<T>::filterSteps(T signal){
    if(_steps > 0)
        for (int i=0; i < _steps; ++i)
            this->filterStep(signal);
}
