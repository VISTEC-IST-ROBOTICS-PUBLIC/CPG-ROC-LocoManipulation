// Created on 05/03/2019 by Carlos
// Adaptive Motor Controller for Compliance Control of a biologial limb.


#include "AdaptiveMotorController.h"

//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

// CLASS OBJECT CONSTRUCTOR
AdaptiveMotorController::AdaptiveMotorController(float beta_value, float a_value, float b_value)
{
    beta = beta_value;
    a = a_value;
    b = b_value;
}


// CLASS OBJECT DESTRUCTOR
AdaptiveMotorController::~AdaptiveMotorController()
{
    cout << "Adaptive Motor Controller destroyed succesfully" << endl;
}


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// MATHEMATICAL METHODS


// Convert encoder values to radians
float AdaptiveMotorController::encToRad(int encoderFeedback)
{
    int diff = encoderFeedback - encoder_pos_init;
    float rad = (float) (2*PI*diff)/encoder_max_value;
    return rad; 
}


// Calculate the positional difference between Qdesired and Qcurrent. Might require conversion from the motor encoder to radians
vector<float> AdaptiveMotorController::getPositionDifference(vector<float> current, vector<float> desired)
{
    vector<float> pos_diff;
    for(size_t i=0; i<current.size(); i++)
    {
        float pos = (current[i] - desired[i]);
        pos_diff.push_back(pos);
    }
    return pos_diff;
}


// Calculate the velocity difference between Qdesired and Qcurrent. Might require conversion from the motor encoder to radians
vector<float> AdaptiveMotorController::getVelocityDifference(vector<float> current, vector<float> desired)
{
    vector<float> vel_diff;
    for(size_t i=0; i<current.size(); i++)
    {
        float vel = (current[i] - desired[i]);
        vel_diff.push_back(vel);
    }
    return vel_diff;
}


// Calculate the acceleration difference between Qdesired and Qcurrent. Might require conversion from the motor encoder to radians
vector<float> AdaptiveMotorController::getAccelerationDifference(vector<float> currentVel, vector<float> previousVel, double dt)
{
    vector<float> acc_diff;
    for(size_t i=0; i<currentVel.size(); i++)
    {
        float acc = (float) (currentVel[i] - previousVel[i])/dt;
        acc_diff.push_back(acc);
    }
    return acc_diff;
}


// Calculation Control Tracking Error (linear combination of position and velocity errors)
vector<float> AdaptiveMotorController::getTrackingError(vector<float> posDiff, vector<float> velDiff)
{
    vector<float> trackError;
    for(size_t i=0; i<posDiff.size(); i++)
    {
        float TE = posDiff[i] + velDiff[i]*beta;
        trackError.push_back(TE);
    }
    return trackError;
}


// Adaptation Scalar to calculate the Force term of the AMC Controller
vector<float> AdaptiveMotorController::approximateGammaCoefficient(vector<float> trackingError)
{
    vector<float> gamma;
    for(size_t i=0; i<trackingError.size(); i++)
    {
        float den = 1 + b*trackingError[i]*trackingError[i];
        float G = a/den;
        gamma.push_back(G);
    }
    return gamma;
}


vector<float> AdaptiveMotorController::calculateGammaCoefficient(vector<float> trackingError)
{
    vector<float> gamma;

    for(size_t i=0; i<trackingError.size(); i++)
    {
        float mag = magnitude(trackingError);
        float den = 1 + b*mag*mag;
        float G = a/den;
        gamma.push_back(G);
    }
    return gamma;
}


// Compute the force in each motor
vector<float> AdaptiveMotorController::getForceTerm(vector<float> trackingError, vector<float> gamma)
{
    vector<float> force;
    for(size_t i=0; i<trackingError.size(); i++)
    {
        float F = trackingError[i]/gamma[i];
        force.push_back(F);
    }
    return force;
}


// Compute the stiffness in each motor
vector<float> AdaptiveMotorController::approximateStiffnessTerm(vector<float> force, vector<float> posDiff)
{
    vector<float> stifness;
    for(size_t i=0; i<force.size(); i++)
    {
        float K = force[i] * posDiff[i];
        stifness.push_back(K);
    }
    return stifness;
}


// Compute the Stiffness matrix
Matrix<float> AdaptiveMotorController::getStiffnessMat(vector<float> force, vector<float> posDiff)
{
    int s = force.size();
    Matrix<float> stiffness(s, s, 0);

    // K(t)[m x m] = F(t) [m x 1] * e(t).t() [1 x m]
    stiffness = stiffness.columnVectorByRowVector(force, posDiff);

    return stiffness;
}


// Compute the damping in each motor
vector<float> AdaptiveMotorController::approximateDampingTerm(vector<float> force, vector<float> velDiff)
{
    vector<float> damping;
    for(size_t i=0; i<force.size(); i++)
    {
        float D = force[i] * velDiff[i];
        damping.push_back(D);
    }
    return damping;
}


// Compute the damping matrix
Matrix<float> AdaptiveMotorController::getDampingMat(vector<float> force, vector<float> velDiff)
{   
    int s = force.size();
    Matrix<float> damping(s, s, 0);

    // D(t)[m x m] = F(t) [m x 1] * e'(t).t() [1 x m]
    damping = damping.columnVectorByRowVector(force, velDiff);

    return damping;
}


// Calculate the final output torque
vector<float> AdaptiveMotorController::approximateTorque(vector<float> force, vector<float> stiffness, vector<float> damping, vector<float> posDiff, vector<float> velDiff)
{
    vector<float> torque;
    for(size_t i=0; i<force.size(); i++)
    {
        float T = -force[i] - stiffness[i]*posDiff[i] - damping[i]*velDiff[i];
        torque.push_back(T);
    }
    return torque;
}


// Calculate final torque using Stiffness and Damping matrices
vector<float> AdaptiveMotorController::calculateTorque(vector<float> force, Matrix<float> stiffness, Matrix<float> damping, vector<float> posDiff, vector<float> velDiff)
{
    vector<float> torque;

    vector<float> s_term = stiffness.matrixByColumnVector(posDiff);
    vector<float> d_term = damping.matrixByColumnVector(velDiff);

    for(size_t i=0; i<force.size(); i++)
    {
        float T = -force[i] - s_term[i] - d_term[i];
        torque.push_back(T);
    }
    return torque;
}


// Removed the derivative part: Tau = -F(t) - K(t)*e(t)
vector<float> AdaptiveMotorController::getProportionalTorque(vector<float> force, vector<float> stiffness, vector<float> posDiff)
{
    vector<float> torque;
    for(size_t i=0; i<force.size(); i++)
    {
        float T = -force[i] - stiffness[i]*posDiff[i];
        torque.push_back(T);
    }
    return torque;
}


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// PARAMETER METHODS

// Get Beta 
float AdaptiveMotorController::getParameterBeta()
{
    return beta;
}


float AdaptiveMotorController::getParameterA()
{
    return a;
}


float AdaptiveMotorController::getParameterB()
{
    return b;
}


int AdaptiveMotorController::getEncoderInit()
{
    return encoder_pos_init;
}


int AdaptiveMotorController::getEncoderMax()
{
    return encoder_max_value;
}


// Set the value of beta
void AdaptiveMotorController::setParameterBeta(float value)
{
    beta = value;
}


// Set a value
void AdaptiveMotorController::setParameterA(float value)
{
    a = value;
}


// Set b value
void AdaptiveMotorController::setParameterB(float value)
{
    b = value;
}


// Set the initial position read by the encoder
void AdaptiveMotorController::setEncoderInit(int value)
{
    encoder_pos_init = value;
}


// Set maximum resolution value read by the encoder
void AdaptiveMotorController::setEncoderMax(int value)
{
    encoder_max_value = value;
}


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// NUMERICAL


float AdaptiveMotorController::magnitude(vector<float> vec)
{
    float sum = 0;

    for(size_t i=0; i<vec.size(); i++)
    {
        sum += vec[i] * vec[i];
    }

    float magnitude = sqrt(sum);

    return magnitude;
}