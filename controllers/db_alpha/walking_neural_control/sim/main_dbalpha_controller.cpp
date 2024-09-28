/*
 * Written by Mathias Thor DEC 30
 * Happy NewYear!
 */

#include "dbalpha_controller.h"

// Main code:
int main(int argc,char* argv[])
{
    std::cout << "Running Main controller" << std::endl;
    dungBeetleController controller(argc,argv);
    while(controller.runController()){}
    return(0);
}
