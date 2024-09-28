//
// Created by mat on 3/4/18.
//

#include "postProcessing.h"

postProcessing::postProcessing() = default;


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// PRIVATE METHODS


// Low pass filter
void postProcessing::lowPassFiltering(double signal) 
{
    // Calculate the amplitudes
    LPFSignal = LPFSignal - (LPFbeta * (LPFSignal - signal));
}

// Amplitude
void postProcessing::calculateAmplitude(double signal)
{
    // Calculate the derivative
    sigDot = (signal - signalOld)/0.04; // 25Hz = 0.04s period

    // See if the derivative changes sign
    if ((((sigDot >= 0) ^ (sigPrimeOld < 0)) == 0) && timeSinceZeroDerivative>12) {
        // If it changes sign then it is a local max- or minimum
        if (sigDot > 0 && !lastZeroDerivative) {
            zeroDerivative[0] = signalOld;
            lastZeroDerivative = true;
        } else if (sigDot < 0 && lastZeroDerivative){
            zeroDerivative[1] = signalOld;
            lastZeroDerivative = false;
        }

        // Subtract local minimum from maximum to get amplitude
        amplitude = std::fabs(zeroDerivative[0]-zeroDerivative[1])/2;

        if(timeSinceZeroDerivative != timeBetweenZeroDerivative)
            timeBetweenZeroDerivative = (timeSinceZeroDerivative+timeBetweenZeroDerivative)/2;

        timeSinceZeroDerivative = 0;
    }
    else {
        timeSinceZeroDerivative++;
    }

    // Advance
    sigPrimeOld = sigDot;
    signalOld = signal;
}



//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// PUBLIC METHODS


// Calculate Low-pass-filtered amplitude
double postProcessing::calculateLPFAmplitude(double signal)
{
    lowPassFiltering(signal);
    calculateAmplitude(LPFSignal);
    return amplitude;
}

// Calculate low-pass-filtered signal
double postProcessing::calculateLPFSignal(double signal)
{
    lowPassFiltering(signal);
    return LPFSignal;
}

// Calculate the amplitude of the signal
double postProcessing::calculateAmplitudeSignal(double signal)
{
    //calculateAmplitude(LPFsignal);
    calculateAmplitude(signal);
    return amplitude;
}

// Set the LPF parameter Beta
void postProcessing::setBeta(double beta) 
{
    LPFbeta=beta;
}

// Get the value of the LPF signal
double postProcessing::getLPFSignal()
{
    return LPFSignal;
}

// Get the amplitude of the signal
double postProcessing::getAmplitudeSignal()
{
    return amplitude;
}


double postProcessing::getTimeBetweenZeroDerivative()
{
    return timeBetweenZeroDerivative;
}