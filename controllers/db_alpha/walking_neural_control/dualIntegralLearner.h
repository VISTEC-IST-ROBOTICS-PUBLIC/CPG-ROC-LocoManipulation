//
// Created by mat on 11/20/17.
//

#ifndef DUNG_BEETLE_CONTROLLER_DUALINTEGRALLEARNER_H
#define DUNG_BEETLE_CONTROLLER_DUALINTEGRALLEARNER_H

#include <vector>
#include <cmath>
#include <iostream>

class dualIntegralLearner {
public:
    dualIntegralLearner(bool _twoSideError, int delayedStart);
    void step(double error, double base_state);

    // accessor functions
    float getControlOutput();
    float getError();
    float getIntegralError();
    float getControllerCorrection();

    // mutator functions
    void setWeights(float,float,float,float);
    void setLowerThreshold(float);

    float trueError = 0;

private:
    float fastLearner = 0;
    float slowLearner = 0;

    float As = 0.72;
    float Iff = 0.0007;

    float Af = 0.5;
    float Bf = 0.08;

    float fastLearnerP = 0;
    float slowLearnerP = 0;
    float fastLearnerI = 0;
    float slowLearnerI = 0;

    float multi = 4.5;
    float Afp = 0.59;
    float Bfp = 0.037*multi;

    float Asp = 0.992;
    float Bsp = 0.0036*multi;

    float Afi = 0.059;
    float Bfi = 0.0018*multi;

    float Asi = 0.0992;
    float Bsi = 0.00018*multi;


//    float fastLearnerP = 0;
//    float slowLearnerP = 0;
//    float fastLearnerI = 0;
//    float slowLearnerI = 0;
//
//    float Afp = 0.2;
//    float Asp = 0.8;
//    float Bfp = 0.2;
//    float Bsp = 0.1;
//
//    float Afi = 0.2;
//    float Asi = 0.7;
//    float Bfi = 0.0014;
//    float Bsi = 0.0007;


    float error = 0, oldError = 0;
    int runs = 0;
    float integralError = 0;
    float lowerThreshold = 0.01;
    float controlOutput = 0;
    float dualLearner = 0;
    bool twoSideError = true;

    double baseState = 0;

    int delayedStart = 150;

};

#endif //DUNG_BEETLE_CONTROLLER_DUALINTEGRALLEARNER_H
