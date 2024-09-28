// ADAPTIVE MOTOR CONTROLLER FOR THE DUNG BEETLE ROBOT
// Created by Carlos on 05/03/2019.

#ifndef DUNG_BEETLE_CONTROLLER_AMC_H
#define DUNG_BEETLE_CONTROLLER_AMC_H


#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include "../alphaMotorDefinition.h"
#include "vaam-II-library/AdaptiveMotorController.h"
#include "vaam-II-library/Matrix.h"

using namespace std;

// Forward declarations
class AdaptiveMotorController;

class AMC
{
    public:

        // Constructor
        AMC(float beta, float a, float b);

        // Destructor
        ~AMC();

        // Public methods
        
        // Controllers
        vector<float> approximateTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel); // Vector forms of K(t) and D(t)
        vector<float> interpolateTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel); // Matrix forms of K(t) and D(t)
        vector<float> porportionalTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel); // Adaptive Proportional K(t) control
        vector<float> getWalkingTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel); //P-Controller with adapted values during stance
        
        // Useful
        vector<float> assignCurrentVector(vector<float> current);
        vector<float> copyVector(vector<float> input);
        float convertEncToRad(int fb);
        void setEncoderParams(int init, int max);
        
        // Mathematical methods
        vector<float> getPosError(vector<float> posDesired, vector<float> pos);
        float optimization(float parameter, float error);
        float degToRad(float deg);
        float radToDeg(float rad);
        float normalize_angle_rad(float rad);
        float normalize_angle_deg(float deg);
        float fabs(float value);
        vector<float> lowPassFilter(float lambda, vector<float> current, vector<float> previous);
        
        // Assignment
        void printVector(vector<float> vec, string name, float magnitude);
        void saveVector(vector<float> vec, ofstream& file, float magnitude);
        void getParams();
        void setA(float a);
        void setB(float b);
        void setBeta(float beta);
        void setConstantCoefficients();

    private:

        // Class pointers
        AdaptiveMotorController * amc;

        // Vectors
        vector<float> pos_error;
        vector<float> vel_error;
        vector<float> track_error;
        vector<float> gamma;
        vector<float> force;
        vector<float> previous_stiffness = {0.5, 0.5, 0.5,
                                            0.5, 0.5, 0.5, 
                                            0.5, 0.5, 0.5,
                                            0.5, 0.5, 0.5,
                                            0.5, 0.5, 0.5,
                                            0.5, 0.5, 0.5,
                                            0.5, 0.5, 0.5};
        vector<float> previous_damping = {0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0};
        vector<float> previous_force = {0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0};
        vector<float> stiffness_k;
        vector<float> force_k;
        // Data files
        ofstream pos_error_csv;
        ofstream vel_error_csv;
        ofstream stiffness_csv;
        ofstream damping_csv;
};

#endif //DUNG_BEETLE_CONTROLLER_AMC_H