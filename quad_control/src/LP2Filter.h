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
     *  - k1, k2: filter's bandwidths
     *  - cond1, cond2: initial conditions
     */
    void filter(T* signal, int length, double k1, double k2, T cond0, T cond2);

    // Set initial condition
    void setInitCond(T cond1, T cond2);

    // Set filter bandwidths
    void setBandwidth(double k1, double k2);

    // Set initial condition and bandwidth
    void initFilterStep(double k1, double k2, T cond1, T cond2);

    /*
     * Compute a single filter step. Before calling this, initialize the filter
     * using initFilterStep(). Results can be retrieved through the functions:
     * lastFiltered(), lastFirst(), lastSecond().
     * Parameters:
     *  - signal: the new value of the signal
     */
    void filterStep(T signal);


private:
    T *_f;                      // Filtered signal
    T *_d, *_dd;                // First and second derivatives
    T _stepF, _stepD, _stepDD;  // Filtered signal and derivatives
    T _intOut1, _intOut2;       // Output of integrals (initial cond at t=0)
    double _k1, _k2;            // Bandwidths

};


#include "LP2Filter.tpp"    // Implementation file

#endif
