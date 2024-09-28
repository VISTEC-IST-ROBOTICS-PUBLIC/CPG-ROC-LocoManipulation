//
// Created by mat on 3/4/18.
//

#ifndef DUNG_BEETLE_CONTROLLER_POSTPROCESSING_H
#define DUNG_BEETLE_CONTROLLER_POSTPROCESSING_H

#include <vector>
#include <cmath>
#include <iostream>

class postProcessing 
{
    public:
        
        // Constructor
        postProcessing();

        // Methods
        double getLPFSignal();
        double getAmplitudeSignal();
        double calculateAmplitudeSignal(double);
        double calculateLPFSignal(double);
        void setBeta(double);
        double calculateLPFAmplitude(double);
        double getTimeBetweenZeroDerivative();

    private:

        // Private Methods
        void calculateAmplitude(double);
        void lowPassFiltering(double);
        
        // Variables
        float sigDot = 0, sigPrimeOld = 0, signalOld = 0;
        int timeSinceZeroDerivative = 0;
        int timeBetweenZeroDerivative = 0;
        bool lastZeroDerivative = false;
        std::vector<float> zeroDerivative = {0,0};
        float amplitude = 0;
        float LPFSignal = 0, LPFbeta = 0.8;
};


#endif //DUNG_BEETLE_CONTROLLER_POSTPROCESSING_H
