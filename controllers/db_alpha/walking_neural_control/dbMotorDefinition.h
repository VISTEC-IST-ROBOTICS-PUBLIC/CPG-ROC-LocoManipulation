//
// Edited by Carlos on 04/30/19.
// From Mathias Thor's Neutron Controller
//

#ifndef DUNG_BEETLE_CONTROLLER_DBMOTORDEFINITION_H
#define DUNG_BEETLE_CONTROLLER_DBMOTORDEFINITION_H


enum dbSensorNames
{
    DB_SENSORAX = 0,
};

enum dbMotorNames
{
    // Legs
    BC2 = 0, // Upward (+), Downward (-)
    CF2 = 1,
    FT2 = 2,
    BC5 = 3,
    CF5 = 4,
    FT5 = 5,
    BC1 = 6,
    CF1 = 7,
    FT1 = 8,
    BC4 = 9,
    CF4 = 10,
    FT4 = 11,
    BC0 = 12,
    CF0 = 13,
    FT0 = 14,
    BC3 = 15,    
    CF3 = 16,
    FT3 = 17,
    
    // Backbone
    LONGITUDINAL = 18,
    TRANSVERSAL = 19,
    
    // Head
    HEAD = 20,

    //Changing according to the maximum motor number
    DB_MOTOR_MAX = 21,
};

#endif //DUNG_BEETLE_CONTROLLER_DBMOTORDEFINITION_H
