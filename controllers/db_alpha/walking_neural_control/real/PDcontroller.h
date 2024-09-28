//
// Created by Carlos on 05/2019
//

#ifndef DUNG_BEETLE_CONTROLLER_PDCONTROLLER_H_
#define DUNG_BEETLE_CONTROLLER_PDCONTROLLER_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>

#define PI 3.14159265359

using namespace std;

class PDcontroller
{
    public:

        // Constructor
        PDcontroller(float k, float d);

        // Destructor
        ~PDcontroller();

        // PUBLIC METHODS

        // Math
        vector<float> getPositionError(vector<float> current, vector<float> desired);
        vector<float> getVelocityError(vector<float> current, vector<float> desired);
        vector<float> calculateOutputTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel);

        // General
        float getStiffnessK();
        float getDampingD();
        void setStiffnessValue(float value);
        void setDampingValue(float value);
        void saveVector(vector<float> vec, ofstream& file, float magnitude);

    private:

        float stiffness = 1.0;
        float damping = 1.0;

        ofstream pos_error_csv;
};

#endif //DUNG_BEETLE_CONTROLLER_PDCONTROLLER_H_
