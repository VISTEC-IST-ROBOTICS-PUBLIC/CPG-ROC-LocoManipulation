//
// Created by mat on 8/26/17.
//

#ifndef NEUTRON_CONTROLLER_NEURONMOTORDEFINITION_H
#define NEUTRON_CONTROLLER_NEURONMOTORDEFINITION_H

enum NeutronSensorNames{
    NEURON_SENSORAX = 0,
};

enum NeutronMotorNames{
    BC0 = 0,
    BC1 = 1,
    BC2 = 2, // Upward (+), Downward (-)
    BC3 = 3,
    BC4 = 4,
    BC5 = 5,

    CF0 = 6,
    CF1 = 7,
    CF2 = 8, // Upward (+), Downward (-)
    CF3 = 9,
    CF4 = 10,
    CF5 = 11,

    FT0 = 12,
    FT1 = 13,
    FT2 = 14, // Upward (+), Downward (-)
    FT3 = 15,
    FT4 = 16,
    FT5 = 17,

	TA = 18,

    //Changing according to the maximum motor number
    NEURON_MOTOR_MAX = 19,
};

#endif //NEUTRON_CONTROLLER_NEURONMOTORDEFINITION_H
