#include "dln.h"

DelayN::DelayN(int size)
{
	/*******************************************************************************
	*  MODULE 2 CPG POST-PROCESSING
	*******************************************************************************/
	buffer.resize(size);
	setNeuronNumber(1);
	step = 0;
//	setOutput(0,0);

}
void DelayN::updateOutputs(){
			//***CPG post processing*****
	  buffer.at(step) = input; //write

	  double y;
	  y = buffer.at(mod(step-delay,buffer.size())); //read

	  step++;
	  if (step % buffer.size() == 0) {
	    step = 0;
	  }

	//***CPG post processing*end*

	setOutput(0,y);

}

int DelayN::mod(int x, int m) {
       int r = x%m;
       return r<0 ? r+m : r;
}

void DelayN::setInput(double _input){
	input = _input;
}

void DelayN::setDelay(int _delay){
	delay = _delay;
}
