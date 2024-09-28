//
// Created by mat on 11/20/17.
//

#include "dualIntegralLearner.h"

#include <cmath>

dualIntegralLearner::dualIntegralLearner(bool _twoSideError, int _delayedStart=0){
    if(!_twoSideError)
        twoSideError = _twoSideError;

    delayedStart=_delayedStart; // 150
}

void dualIntegralLearner::step(double error, double _baseState) {

    baseState = (error == 0) ? _baseState : baseState; // used in 4,5,6

    if(error == 0){
        error = 0;
        oldError = 0;
        integralError = 0;
    }

    // Delayed start of learner (due to transient phase)
    if( runs > delayedStart ) {
        oldError = error;
        trueError = error;

        // If we have no positive error and we are above the base case and we also react to
        // negative error then stay at base state!
        if(error <= 0 && controlOutput >= baseState && twoSideError) { // used in vanilla, freq, dual
            error = 0;
            oldError = 0;
            integralError = 0;
            controlOutput = baseState;
        }

        // If we do not react to negative errors (cannot rise)
        if(error < 0 && !twoSideError) { // used in 3,4,5,6
            error = 0;
            oldError = 0;
            integralError = 0;
        }

        // If baseState = 0, then do not integrate error
        if(baseState == 0) { // used in 3
            integralError = 0;
        }

    }
    else {
        runs++;
        baseState = _baseState;
        error = 0;
        oldError = 0;
        integralError = 0;
    }

    /* Simple Learner (to be tested) */
    // controlOutput = baseState - learningRate*error;

    // Dual-Learner STANDARD
//    fastLearner = (Af*fastLearner) + (Bf*error);
//    slowLearner = (As*slowLearner) + (Bs*error);
//    dualLearner = fastLearner + slowLearner;

    // Dual-Learner NEW v.1
//    integralError += error;
//    fastLearner = Iff*integralError + Gf*error;
//    slowLearner = Is*integralError + Gs*error;
//    dualLearner = fastLearner + slowLearner;

    // Dual-Learner v.2
//    integralError += error;
//    slowLearner = (As*slowLearner) + ((Iff)*integralError);
//    fastLearner = (Af*fastLearner) + (Bf*error);
//    dualLearner = fastLearner + slowLearner;

    // Dual-Learner v.3
//    integralError += error;
//    fastLearnerP = (Afp*fastLearner) + (Bfp*error);
//    slowLearnerP = (Asp*slowLearner) + (Bsp*error);
//    fastLearnerI = (Afi*fastLearner) + (Bfi*integralError);
//    slowLearnerI = (Asi*slowLearner) + (Bsi*integralError);
//    dualLearner = fastLearnerP + slowLearnerP + fastLearnerI + slowLearnerI;

    // Dual-Learner v.4
    integralError += error;
    fastLearnerP = (Afp*fastLearner) + (Bfp*error) + (Bfi*integralError);
    slowLearnerP = (Asp*slowLearner) + (Bsp*error) + (Bsi*integralError);
    dualLearner = fastLearnerP + slowLearnerP;

    // Calculate controlOutput
    controlOutput = baseState - dualLearner;

    // The output can not be lower than some Threshold
    if (controlOutput < lowerThreshold) {
        controlOutput = lowerThreshold;
        integralError -= error;
    }
}

float dualIntegralLearner::getControlOutput() {
    return controlOutput;
}

float dualIntegralLearner::getControllerCorrection() {
    return dualLearner;
}

float dualIntegralLearner::getError() {
    return error;
}

//void dualIntegralLearner::setWeights(float _IIf, float _Gf, float _Is, float _Gs){
//    Iff = _IIf; // Small integrator
//    Gf = _Gf; // Small gain
//    Is = _Is; // Big integrator
//    Gs = _Gs; // Big gain
//}

void dualIntegralLearner::setLowerThreshold(float _lowerThreshold) {
    lowerThreshold=_lowerThreshold;
}

float dualIntegralLearner::getIntegralError() {
    return integralError;
}
