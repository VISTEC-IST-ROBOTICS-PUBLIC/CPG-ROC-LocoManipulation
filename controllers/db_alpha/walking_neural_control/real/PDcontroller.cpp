//
// Created by Carlos on 05/2019
//


#include "PDcontroller.h"


//------------------------------------------------------------------------------------------


PDcontroller::PDcontroller(float k, float d)
{
    stiffness = k;
    damping = d;

    // Do not forget to change paths. 
    // General instalation space of gorobots: /home/user/workspace/gorobots
    // Maybe "/home/user/workspace/dung_beetle_experiments/*.csv"
    pos_error_csv.open("/home/charlie/Workspace/AI/MasterThesis/experiments/position_error_rad.csv");
}

PDcontroller::~PDcontroller()
{
    pos_error_csv.close();
}


//------------------------------------------------------------------------------------------


// PUBLIC METHODS


// Math


vector<float> PDcontroller::getPositionError(vector<float> current, vector<float> desired)
{
    vector<float> pos_diff;
    for(size_t i=0; i<current.size(); i++)
    {
        float pos = (current[i] - desired[i]);
        pos_diff.push_back(pos);
    }
    return pos_diff;
}


vector<float> PDcontroller::getVelocityError(vector<float> current, vector<float> desired)
{
    vector<float> vel_diff;
    for(size_t i=0; i<current.size(); i++)
    {
        float vel = (current[i] - desired[i]);
        vel_diff.push_back(vel);
    }
    return vel_diff;
}


vector<float> PDcontroller::calculateOutputTorque(vector<float> currentPos, vector<float> desiredPos, vector<float> currentVel, vector<float> desiredVel)
{
    vector<float> error_pos = getPositionError(currentPos, desiredPos);
    vector<float> error_vel = getVelocityError(currentVel, desiredVel);
    
    vector<float> torque;
    for(size_t i=0; i<error_pos.size(); i++)
    {
        float t = stiffness * error_pos[i] + damping * error_vel[i];
        torque.push_back(t);
    }

    saveVector(error_pos, pos_error_csv, 1);

    return torque;
}


// General


float PDcontroller::getStiffnessK()
{
    return stiffness;
}


float PDcontroller::getDampingD()
{
    return damping;
}


void PDcontroller::setStiffnessValue(float value)
{
    stiffness = value;
}


void PDcontroller::setDampingValue(float value)
{
    damping = value;
}

// Save vector to file in CSV format
void PDcontroller::saveVector(vector<float> vec, ofstream& file, float magnitude)
{
    file << vec[0]*magnitude;
    for(size_t i=1; i<vec.size(); i++)
    {
        file << ", " << vec[i]*magnitude;
    }
    file << endl;
}