#ifndef _LP_2_FILTER_
#define _LP_2_FILTER_

/*
 * This represents a low-pass, 2nd order filter, used for computing first and
 * second derivative of a signal.
 * It is possible to filter a whole signal, or filtering step-by-step.
 */
template <typename T> class LP2Filter{

public:
    LP2Filter();
    ~LP2Filter();

    // Return the filtered signal
    T* filtered(){ return _f; }

    // Return the signal first derivative
    T* first(){ return _d; }

    // Return the signal second derivative
    T* second(){ return _dd; }

    // Return the last computed value of the filtered signal
    T lastFiltered(){ return _stepF; }

    // Return the last computed value of the first derivative
    T lastFirst(){ return _stepD; }

    // Return the last computed value of the first derivative
    T lastSecond(){ return _stepDD; }

    /*
     * Filter a signal. Results can be retrieved through the functions:
     * filtered(), first(), second().
     * Parameters:
     *  - signal: the signal to filter, as a pointer to the first element of the array
     *  - length: number of samples in the signal array
     *  - sampleTime: signal sample time
     *  - k1, k2: filter's bandwidths
     *  - cond1, cond2: initial conditions
     */
    void filter(T* signal, unsigned int length, double sampleTime, double k1,
            double k2, T cond1, T cond2, int steps=1);

    // Set initial condition
    void setInitCond(T cond1, T cond2);

    // Set filter bandwidths
    void setBandwidth(double k1, double k2);

    // Set initial condition and bandwidth
    void initFilterStep(double sampleTime, double k1, double k2, T cond1,
        T cond2, int steps=1);

    // Set filter sample time
    void setSampleTime(double t);

    // Reset the filter.
    void reset();

    /*
     * Compute a single filter step. Before calling this, initialize the filter
     * using initFilterStep(). Results can be retrieved through the functions:
     * lastFiltered(), lastFirst(), lastSecond().
     * Parameters:
     *  - signal: the new value of the signal
     */
    void filterStep(T signal);

    /*
     * Filter signal value 'steps' times.
     * Greater steps values leads to smoother outputs.
     */
    void filterSteps(T signal);


private:
    T *_f;                      // Filtered signal
    T *_d, *_dd;                // First and second derivatives
    T _stepF, _stepD, _stepDD;  // Filtered signal and derivatives
    T _x1, _x2;                 // Integrator state
    T _init1, _init2;           // Integrator initial state
    T _intOut2;                 // Output of the second integrator
    double _k1, _k2;            // Bandwidths
    double _t;                  // Sample time
    int _steps;
    bool _first;                // True if it's the first computation

};


#include "LP2Filter.tpp"    // Implementation file

#endif
