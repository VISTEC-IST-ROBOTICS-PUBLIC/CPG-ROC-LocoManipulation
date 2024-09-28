// ADAPTIVE MOTOR CONTROLLER FOR THE DUNG BEETLE ROBOT
// Created by Carlos on 05/03/2019.

#include "AMC.h"

//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// Constructor
AMC::AMC(float beta, float a, float b)
{
    // Create controller object
    amc = new AdaptiveMotorController(beta, a, b);
    
    // Do not forget to change paths. 
    // General instalation space of gorobots: /home/user/workspace/gorobots
    // Maybe "/home/user/workspace/dung_beetle_experiments/*.csv"
    pos_error_csv.open("/home/binggwong/experiments/position_error_rad.csv");
    vel_error_csv.open("/home/binggwong/experiments/velocity_error_rad_s.csv");
    stiffness_csv.open("/home/binggwong/experiments/stiffness_Nrad.csv");
    damping_csv.open("/home/binggwong/experiments/damping_Nrad_s.csv");
}


// Destructor
AMC::~AMC()
{
    stiffness_csv.close();
    damping_csv.close();
    pos_error_csv.close();
    vel_error_csv.close();
}


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// Controller v4.0: Removed the derivative part.
vector<float> AMC::porportionalTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel)
{
    // Create output vector
    vector<float> torques;
    float lambda = 0.04;

    // Clear vectors from last iteration
    pos_error.clear();
    vel_error.clear();
    track_error.clear();
    gamma.clear();
    force.clear();

    // Calculate errors
    pos_error = amc->getPositionDifference(currentPos, desiredPos);
    vel_error = amc->getVelocityDifference(currentVel, desiredVel);
    track_error = amc->getTrackingError(pos_error, vel_error);
    

    // Calculate gamma
    gamma = amc->approximateGammaCoefficient(track_error);

    // Calculate terms
    force = amc->getForceTerm(track_error, gamma);
    
    // Calculate and filter Stiffness:
    vector<float> stiffness = amc->approximateStiffnessTerm(force, pos_error);
    stiffness = lowPassFilter(lambda, stiffness, previous_stiffness);
    
    // Calculate output torque
    torques = amc->getProportionalTorque(force, stiffness, pos_error);

    // Output AMC Data
    saveVector(pos_error, pos_error_csv, 1);
    saveVector(stiffness, stiffness_csv, 1);
    
    // Update previous vectors:
    previous_stiffness.clear();
    previous_force.clear();
    previous_stiffness = copyVector(stiffness);
    previous_force = copyVector(force);

    return torques;
}


// Controller v2.0: Vector form of K(t) and D(t)
vector<float> AMC::approximateTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel)
{
    // Create output vector
    vector<float> torques;
    float lambda = 0.05;

    // Clear vectors from last iteration
    pos_error.clear();
    vel_error.clear();
    track_error.clear();
    gamma.clear();
    force.clear();

    // Calculate errors
    pos_error = amc->getPositionDifference(currentPos, desiredPos);
    vel_error = amc->getVelocityDifference(currentVel, desiredVel);
    track_error = amc->getTrackingError(pos_error, vel_error);
    

    // Calculate gamma
    gamma = amc->approximateGammaCoefficient(track_error);

    // Calculate terms
    force = amc->getForceTerm(track_error, gamma);
    
    // Calculate and filter Stiffness:
    vector<float> stiffness_raw = amc->approximateStiffnessTerm(force, pos_error);
    vector<float> stiffness = lowPassFilter(lambda, stiffness_raw, previous_stiffness);
    
    vector<float> damping_raw = amc->approximateDampingTerm(force, vel_error);
    vector<float> damping = lowPassFilter(lambda, damping_raw, previous_damping);
    
    // Calculate output torque
    torques = amc->approximateTorque(force, stiffness, damping, pos_error, vel_error);

    // Output AMC Data
    saveVector(pos_error, pos_error_csv, 1);
    saveVector(stiffness, stiffness_csv, 1);
    saveVector(damping, damping_csv, 1);
    saveVector(vel_error, vel_error_csv, 1);
    
    // Update previous vectors:
    previous_stiffness.clear();
    previous_damping.clear();
    previous_stiffness = copyVector(stiffness);
    previous_damping = copyVector(damping);

    return torques;
}


// Controller v1.0: Matrix form of K(t) and D(t)
vector<float> AMC::interpolateTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel)
{
    // Create output vector
    vector<float> torques;

    // Clear vectors from last iteration
    pos_error.clear();
    vel_error.clear();
    track_error.clear();
    gamma.clear();
    force.clear();

    // Calculate errors
    pos_error = amc->getPositionDifference(currentPos, desiredPos);
    vel_error = amc->getVelocityDifference(currentVel, desiredVel);
    track_error = amc->getTrackingError(pos_error, vel_error);
    
    // Calculate gamma
    //gamma = amc->getGammaCoefficient(track_error);
    gamma = amc->calculateGammaCoefficient(track_error);
    
    // Calculate terms
    force = amc->getForceTerm(track_error, gamma);
    Matrix<float> stiffness = amc->getStiffnessMat(force, pos_error);
    Matrix<float> damping = amc->getDampingMat(force, vel_error);

    // Calculate output torque
    torques = amc->calculateTorque(force, stiffness, damping, pos_error, vel_error);

    // Output AMC Data
    return torques;
}


// v5.0 Experimental version
vector<float> AMC::getWalkingTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel)
{
    vector<float> torques;
    float lambda = 0.04;

    // Clear vectors from last iteration
    pos_error.clear();
    vel_error.clear();
    track_error.clear();
    gamma.clear();

    // Calculate errors
    pos_error = amc->getPositionDifference(currentPos, desiredPos);
    vel_error = amc->getVelocityDifference(currentVel, desiredVel);
    track_error = amc->getTrackingError(pos_error, vel_error);

    // Calculate gamma
    gamma = amc->approximateGammaCoefficient(track_error);

    // Calculate F(t) and K(t)
    vector<float> F = amc->getForceTerm(track_error, gamma);
    vector<float> K = amc->approximateStiffnessTerm(F, pos_error);
    K = lowPassFilter(lambda, K, previous_stiffness);
    
    // Filter F(t)
    for(size_t i; i<F.size(); i++)
    {
        if(F[i] <= force_k[i])
        {
            F.at(i) = F[i] + force_k[i];
        }
        else
        {
            F.at(i) = F[i];
        }
    }

    // Filter K(t)
    for(size_t j; j<K.size(); j++)
    {
        if(F[j] <= stiffness_k[j])
        {
            K.at(j) = K[j] + stiffness_k[j];
        }
        else
        {
            K.at(j) = K[j];
        }
    }

    // Calculate torque
    torques = amc->getProportionalTorque(F, K, pos_error);

    // Update
    previous_stiffness.clear();
    previous_stiffness = copyVector(K);

    // Output AMC Data
    saveVector(pos_error, pos_error_csv, 1);
    saveVector(K, stiffness_csv, 1);

    return torques;
}


//----------------------------------------------------------------------------------------------------------------------------------


// Update current velocities
vector<float> AMC::assignCurrentVector(vector<float> current)
{   
    vector<float> feedback;
    feedback.resize(12);

    feedback.at(0) = current.at(CF2);
    feedback.at(1) = current.at(FT2);
    feedback.at(2) = current.at(CF5);
    feedback.at(3) = current.at(FT5);
    feedback.at(4) = current.at(CF1);
    feedback.at(5) = current.at(FT1);
    feedback.at(6) = current.at(CF4);
    feedback.at(7) = current.at(FT4);
    feedback.at(8) = current.at(CF0);
    feedback.at(9) = current.at(CF0);
    feedback.at(10) = current.at(CF3);
    feedback.at(11) = current.at(FT3);
}

vector<float> AMC::copyVector(vector<float> input)
{
    vector<float> output;
    for(size_t i=0; i<input.size(); i++)
    {
        output.push_back(input[i]);
    }
    return output;
}



// Call to encoderToRadians conversion
float AMC::convertEncToRad(int fb)
{
    return amc->encToRad(fb);
}


// Set encoder maximum value and initial position
void AMC::setEncoderParams(int init, int max)
{
    amc->setEncoderInit(init);
    amc->setEncoderMax(max);
}


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// Calculate and save position error
vector<float> AMC::getPosError(vector<float> posDesired, vector<float> pos)
{
    vector<float> error;
    error = amc->getPositionDifference(pos, posDesired);
    saveVector(error, pos_error_csv, 1);
    return error;
}


// Optimization of AMC parameters
float AMC::optimization(float parameter, float error)
{
    float new_parameter;
    if(error < 0.01)
    {
        new_parameter = parameter;
    }
    else
    {
        new_parameter = parameter + 0.0001;
    }
    return new_parameter;
}


// Convert Degrees to radians
float AMC::degToRad(float deg)
{
    return (deg*2*PI)/360; 
}


// Convert radians to degrees
float AMC::radToDeg(float rad)
{
    return (rad*360)/(2*PI);
}


// Resize angles to range [0, 2*PI)
float AMC::normalize_angle_rad(float rad)
{
    
    float x = fmod(rad, 2*PI);
    //float x = remainder(deg, 360);
    if (x < 0)
        x += 2*PI;
    return x;
}


// Resize angles to range [0, 360)
float AMC::normalize_angle_deg(float deg)
{
    float x = fmod(deg, 360);
    //float x = remainder(deg, 360);
    if (x < 0)
        x += 360;
    return x;
}


// Return absolute value (for floats)
float AMC::fabs(float value)
{
    if(value >= 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


// Low pass filter
vector<float> AMC::lowPassFilter(float lambda, vector<float> current, vector<float> previous)
{
    vector<float> result;
    for(size_t i=0; i<current.size(); i++)
    {
        float lp = (1-lambda)*previous[i] + lambda*current[i];
        result.push_back(lp);
    }
    return result;
}


//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// Print vector in command line
void AMC::printVector(vector<float> vec, string name, float magnitude)
{
    cout << name << ":  " << vec[0]*magnitude;
    for(size_t i=1; i<vec.size(); i++)
    {
        cout << ", " << vec[i]*magnitude;
    }
    cout << endl << endl;
}


// Save vector to file in CSV format
void AMC::saveVector(vector<float> vec, ofstream& file, float magnitude)
{
    file << vec[0]*magnitude;
    for(size_t i=1; i<vec.size(); i++)
    {
        file << ", " << vec[i]*magnitude;
    }
    file << endl;
}


// Print AMC's internal parameters in the command line
void AMC::getParams()
{
    cout << "AMC parameters:" << endl;
    cout << "   >> Beta: " <<  amc->getParameterBeta() << endl;
    cout << "   >> Parameter a: " <<  amc->getParameterA() << endl;
    cout << "   >> Parameter b: " <<  amc->getParameterB() << endl;
    cout << "   >> Encoder's initial position: " <<  amc->getEncoderInit() << endl;
    cout << "   >> Encoder's maximum value: " <<  amc->getEncoderMax() << endl;
}


// Set paramter a dynamically
void AMC::setA(float a)
{
    amc->setParameterA(a);
}


// Set parameter b dynamically
void AMC::setB(float b)
{
    amc->setParameterB(b);
}


// Set parameter beta dynamically
void AMC::setBeta(float beta)
{
    amc->setParameterBeta(beta);
}

void AMC::setConstantCoefficients()
{
    stiffness_k = copyVector(previous_stiffness);
    force_k = copyVector(previous_force);
}
