//
// Edited by BGL on 27/06/2019.
// From db_alpha Modular Controller by Carlos Viescas Huerta.
//

#ifndef DBALPHA_CONTROLLER_MODULARCONTROLLER_H
#define DBALPHA_CONTROLLER_MODULARCONTROLLER_H

#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <string.h>
#include "ann-framework/ann.h"
#include "dbMotorDefinition.h"
#include "./db_pcpg.h"
#include "./db_psn.h"
#include "delayline.h"
#include "dln.h"
#include "ann-library/so2cpg.h"
#include "ann-library/vrn.h"
#include "ann-library/pmn.h"
#include "ann-framework/neuron.h"
#include "ann-library/adaptiveso2cpgsynplas.h"

using namespace std;

// forward declarations
class SO2CPG;
class PCPG; // postCPG / PCPG
class PCPGtest; // postCPG / PCPG
class AdaptiveSO2CPGSynPlas;
class PSN;
class PMN;
class VRN;
class TransferFunction;
class Delayline;
class DelayN;


class modularController: public ANN {
public:
    // Modular Controller Constructor
    modularController(int _cpg_option, bool is_decoupled);

    // MNN Methods
    void step() override;
    double getCpgOutput(int output);
    double getCpgActivity(int output);
    double getpcpgOutput1(int output);
    double getpcpgOutput2(int output);
    double getOutputtau_vrn3(int output);
    double getPsnOutput(int output);
    double getVrnLeftOutput(int output);
    double getVrnRightOutput(int output);
    double getVrnOutput1(int output);
    double getVrnOutput2(int output);
    double getVrnOutput3(int output);
    double getVrnOutput4(int output);
    double getPsnOutput0(int output);
    double getpmnOutput(int output);
    void setPsnInputNeurons(int input, double value);
    void setVrnInputNeurons(int input, double value);

    void setCpgOutput(int neuron, double value);
    void setCpgInputNeuron(double value);
    void setInputVrnLeft(int input, double value);
    void setInputVrnRight(int input, double value);
    void setInputPsn(int input, double value);
    void setInputNeuronInput(int input, double value);

    double getCpgWeight(int neuron1, int neuron2);
    double getCpgBias(int neuron);
    double getFinalNeuronOutput(int output);
    void setPerturbation(double value);

    void setMI(double value);
    double getMI();
    double getPhi();
    void setPhii(double value);

    // Variables
    double MI;
    double cpg_bias;

    // Signal processing methods
    float truncateSignal(float value, float lim, int direction);
    float trimSignal(float value, float prev, float min, int direction);

    float threshold1 = 0.8;
    float threshold2 = 0.5;

private:

    // ANN Objects
    int cpg_option = 0;
    bool isDecoupled;
    SO2CPG * cpg;
    AdaptiveSO2CPGSynPlas * cpg_s;
    PCPG * pcpg; // postCPG / PCPG
    PCPGtest * pcpgc1; // postCPG / PCPG
    PCPGtest * pcpgc2; // postCPG / PCPG
    VRN * vrn1;
    VRN * vrn2;
    VRN * vrn3;
    VRN * vrn4;
    PSN * psn0;
    PSN * psn1;
    PSN * psn2;
    PSN * psn3;
    PSN * psn4;
    PSN * psn5;
	PMN* pmn;
	PSN* psn;
	VRN* vrnLeft;
	VRN* vrnRight;
    DelayN* tau_vrn3;
    DelayN* tau_vrn4;


    // Variables
    std::vector<Neuron*> inputNeurons;
    std::vector<Neuron*> hiddenNeurons;
    std::vector<Neuron*> psnInputNeurons;
    std::vector<Neuron*> vrnInputNeurons;
    std::vector<Neuron*> pcpgDelayNeurons;
    std::vector<Neuron*> cpgInputNeuron;
    Neuron* perturbationNeuron;
};

#endif //DBALPHA_CONTROLLER_MODULARCONTROLLER_H
