// Created on 05/03/2019 by Carlos
// Adaptive Motor Controller for Compliance Control of a biologial limb.
// toque(t) = -F(t) - K(t)*e(t) -D(t)*e'(t)
/* 
Constant values:

 beta = 0.05
 a = 35
 b = 25

*/

#ifndef ADAPTIVEMOTORCONTROLLER_H_
#define ADAPTIVEMOTORCONTROLLER_H_

#include <vector>
#include <math.h>
#include <iostream>
#include "Matrix.h"

#define PI 3.14159265359

using namespace std;

class AdaptiveMotorController
{

    public:

        // Constructor
        AdaptiveMotorController(float beta_value, float a_value, float b_value);

        // Destructor
        ~AdaptiveMotorController();

        // Public Methods

        // Conversion from encoder values to radians
        float encToRad(int encoderFeedback);

        // Position, Velocity, Acceleration and Tracking errors (calculated for each motor).
        vector<float> getPositionDifference(vector<float> current, vector<float> desired); // Vector
        vector<float> getVelocityDifference(vector<float> current, vector<float> desired); // Vector
        vector<float> getAccelerationDifference(vector<float> currentVel, vector<float> previousVel, double dt); //Vector
        vector<float> getTrackingError(vector<float> posDiff, vector<float> velDiff); // Vector
        
        // Gamma Adaptation coefficient.
        vector<float> approximateGammaCoefficient(vector<float> trackingError); // Approximation
        vector<float> calculateGammaCoefficient(vector<float> trackingError); // Magnitude approach

        // Force term
        vector<float> getForceTerm(vector<float> trackingError, vector<float> gamma);

        // Impedance term
        vector<float> approximateStiffnessTerm(vector<float> force, vector<float> posDiff); // Vector 
        Matrix<float> getStiffnessMat(vector<float> force, vector<float> posDiff); // Matrix
        vector<float> approximateDampingTerm(vector<float> force, vector<float> velDiff); // Vector
        Matrix<float> getDampingMat(vector<float> force, vector<float> velDiff); // Matrix

        // Compute the control Torque
        vector<float> approximateTorque(vector<float> force, vector<float> stiffness, vector<float> damping, vector<float> posDiff, vector<float> velDiff); // Vector form
        vector<float> calculateTorque(vector<float> force, Matrix<float> stiffness, Matrix<float> damping, vector<float> posDiff, vector<float> velDiff); // Matrix (Original version from Xiaofeng's paper)
        vector<float> getProportionalTorque(vector<float> force, vector<float> stiffness, vector<float> posDiff); // Just consider the proportional part of the controler: tau = -F(t) - K(t)*e(t)

        // Parameter check methods
        float getParameterBeta();
        float getParameterA();
        float getParameterB();
        int getEncoderInit();
        int getEncoderMax();

        // Parameter setup methods
        void setParameterBeta(float value);
        void setParameterA(float value);
        void setParameterB(float value);
        void setEncoderInit(int value);
        void setEncoderMax(int value);

        // Numerical
        float magnitude(vector<float> vec); // Magnitude of a vector: ||vec||^2

    private:

        // Variables
        float beta = 0;
        float a = 0;
        float b = 0;
        int encoder_pos_init = 0;
        int encoder_max_value = 0;
};

#endif /* ADAPTIVEMOTORCONTROLLER_H_ */