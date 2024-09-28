#ifndef DelayN_H_
#define DelayN_H_

#include "ann-framework/ann.h"
#include <vector>
#include <iostream>
#include <cstdlib>

class DelayN : public ANN {
public:

	double Read(int delay);
    void Write(double input);
    void setInput(double input);
    void setDelay(int delay);
    void Step();
    void Reset();
    static int mod(int x, int m);
    vector<double> buffer;
    int step;

	DelayN(int size);
    void updateOutputs();
//    void setThreshold(double _threshold);
private:
    double input;
    int delay = 0;
};


#endif /* DelayN_H_ */
