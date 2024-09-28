// Main Hexapod Controller run script for Dung Beetle robot

#include "dungBeetleController.h"

// Main code:
int main(int argc,char* argv[])
{
    dungBeetleController controller(argc,argv);
    while(controller.runController()){}
    return(0);
}
