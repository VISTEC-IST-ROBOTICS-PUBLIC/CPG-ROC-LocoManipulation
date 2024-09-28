#ifndef PCPG_H_
#define PCPG_H_

#include "ann-framework/ann.h"


class PCPGtest : public ANN {
public:

	PCPGtest();
    void updateOutputs();
    void setThreshold(double _threshold);
private:
	std::vector<double> set;
	std::vector<double> countup;
	std::vector<double> countdown;

	std::vector<double> pcpg_step;
				std::vector<double> setold;
				std::vector<double> countupold;
				std::vector<double> countdownold;
				std::vector<double> diffset ;
				std::vector<double> deltaxdown;
				std::vector<double> deltaxup;
				std::vector<double> xup;
				std::vector<double> xdown;

				std::vector<double> yup;
				std::vector<double> ydown;
				std::vector<double> pcpg_output;
				double threshold = 0.85;
};


#endif /* PCPG_H_ */
