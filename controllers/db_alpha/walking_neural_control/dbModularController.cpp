//
// Edited by BGL on 27/06/2019.
// From db_alpha Modular Controller by Carlos Viescas Huerta.
//

#include "dbModularController.h"


modularController::modularController(int _cpg_option, bool is_decoupled){
    cpg_option=_cpg_option;
    isDecoupled = is_decoupled;
    /*******************************************************************************
    *  MODULE 0 IO'S for modularController
    *******************************************************************************/

    // Create 5 input neurons for modularController
//    for(int i=0;i<5;i++){
//        inputNeurons.push_back(addNeuron());
//        inputNeurons.at(i)->setTransferFunction(identityFunction());
//    }

    // Create 4 input neurons for modularController
//    for(int i=0;i<4;i++){
//        inputNeurons.push_back(addNeuron());
//        inputNeurons.at(i)->setTransferFunction(identityFunction());
//    }

    // Create 6 input neurons for modularController
//    for(int i=0;i<6;i++){
//        hiddenNeurons.push_back(addNeuron());
//        hiddenNeurons.at(i)->setTransferFunction(identityFunction());
//    }

    for(int i=0;i<6;i++){
    	psnInputNeurons.push_back(addNeuron());
    	psnInputNeurons.at(i)->setTransferFunction(identityFunction());
    }

    for(int i=0;i<4;i++){
        vrnInputNeurons.push_back(addNeuron());
        vrnInputNeurons.at(i)->setTransferFunction(identityFunction());
    }

    for(int i=0;i<2;i++){
    	pcpgDelayNeurons.push_back(addNeuron());
    	pcpgDelayNeurons.at(i)->setTransferFunction(identityFunction());
    }
    for(int i=0;i<2;i++){
    	cpgInputNeuron.push_back(addNeuron());
    	cpgInputNeuron.at(i)->setTransferFunction(identityFunction());
    }
   pcpgDelayNeurons[0]->setInput(1);
    pcpgDelayNeurons[1]->setInput(1);

    cpgInputNeuron[0]->setInput(0);
    cpgInputNeuron[1]->setInput(0);
//    // Create 6 input neurons for modularController
//    for(int i=0;i<6;i++){
//        hiddenNeurons.push_back(addNeuron());
//        hiddenNeurons.at(i)->setTransferFunction(identityFunction());
//    }

    /*******************************************************************************
    *  MODULE 1 CPG
    *******************************************************************************/

    switch(cpg_option) {
        case 1: // Standard SO2 CPG

            cpg = new SO2CPG();
            cpg_bias = 0.0;

            //From 0.02-1.5
            // MI = 0.02;   // slow Wave
            // MI = 0.03;   // Wave
            // MI = 0.04;   // fast Wave
            // MI = 0.13;   // Terapod
            // MI = 0.2;      // Tripod fast
            // MI = 0.34;   // Faster than tripod
            MI = 0.01;     // Rolling Behavior initialization
            //destabilize cpg to oscillate
            cpg->setOutput(0, 0.1);
            cpg->setOutput(1, 0.1);
            cpg->setActivity(0, 0.1);
            cpg->setActivity(1, 0.1);

            //set cpg weights
            cpg->setWeight(0, 0, 1.4);
            cpg->setWeight(0, 1, 0.18 + MI);
            cpg->setWeight(1, 0,-0.18 - MI);
            cpg->setWeight(1, 1, 1.4);

            //set bias
            cpg->setBias(0, cpg_bias);
            cpg->setBias(1, cpg_bias);

            // cpg Input perturbation
            w(cpg->getNeuron(0),cpgInputNeuron[0], 1);

            //for updating the sub nets (to do the time step)
            addSubnet(cpg);
            break;

        case 2: // Adaptive SO2 CPG

            cpg_s = new AdaptiveSO2CPGSynPlas();
            cpg_s->setPhi(0.04);         // Frequency term - Influences w00 w01 w10 w11 of the SO(2) oscillator (long term)
            cpg_s->setEpsilon ( 0.1 );   // Value should depend on the initial and external freq - from P to h2 (short term)
            cpg_s->setAlpha(1.01);
            cpg_s->setGamma   ( 1.0 );    // Synaptic weight from h2 to h0 - Governed by a hebbian-type learning (short term)
            cpg_s->setBeta    ( 0.0 );    // Synaptic weight from h0 to h2 - Governed by a hebbian-type learning (short term)
            cpg_s->setMu      ( 1.0 );    // Learning rate - Value should depend on the given initial and external freq
            cpg_s->setBetaDynamics   ( -1.0, 0.010, 0.00); // Heppian Rate, Decay Rate, Beta_0
            cpg_s->setGammaDynamics  ( -1.0, 0.010, 1.00); // --- || ---
            cpg_s->setEpsilonDynamics(  1.0, 0.010, 0.01); // --- || ---

            //destabilize cpg to oscillate
            cpg_s->setOutput(0,0.2);

            //for updating the sub nets (to do the time step)
            addSubnet(cpg_s);
            break;

        case 3: // Standard SO2 CPG with Phi instead of MI
            cpg = new SO2CPG();
            cpg_bias = 0.0;

            //destabilize cpg to oscillate
            cpg->setOutput(0, 0.1);// cpg->setOutput(0, 0.1);
            cpg->setOutput(1, 0.1);
            cpg->setActivity(0, 0.1);
            cpg->setActivity(1, 0.1);

            //set bias
            cpg->setBias(0, cpg_bias);
            cpg->setBias(1, cpg_bias);

            //for updating the sub nets (to do the time step)
            addSubnet(cpg);

            break;

        default:break;
    };

    /*******************************************************************************
	*  EXTRA MODULE PCPGc1, PCPGc2 // Makes it sawtooth
	*******************************************************************************/

    if (is_decoupled){
        pcpgc1 = new PCPGtest();
        pcpgc2 = new PCPGtest();

        pcpgc1->setThreshold(threshold1);
        pcpgc2->setThreshold(threshold2);

        w(pcpgc1->getNeuron(0),cpg->getNeuron(0),1);
        w(pcpgc2->getNeuron(0),cpg->getNeuron(0),1);

        addSubnet(pcpgc1);
        addSubnet(pcpgc2);        
    }
    else{  
        pcpgc1 = new PCPGtest();
        pcpgc2 = new PCPGtest();

        pcpgc1->setThreshold(0.8);
        pcpgc2->setThreshold(0.5);

        w(pcpgc1->getNeuron(0),cpg->getNeuron(0),1);
        w(pcpgc1->getNeuron(1),cpg->getNeuron(0),-1);

        w(pcpgc2->getNeuron(0),cpg->getNeuron(0),1);
        w(pcpgc2->getNeuron(1),cpg->getNeuron(0),-1);

        addSubnet(pcpgc1);
        addSubnet(pcpgc2);
    }


    /*******************************************************************************
	*  MODULE Delay line
	*******************************************************************************/
    if (is_decoupled){
        tau_vrn3 = new DelayN(10);
        tau_vrn3->setDelay(3);
        tau_vrn3->setInput(pcpgc2->getOutput(0));

        addSubnet(tau_vrn3);
    }
    else{  
        tau_vrn3 = new DelayN(10);
        tau_vrn3->setDelay(3);
        tau_vrn3->setInput(pcpgc2->getOutput(0));
        tau_vrn4 = new DelayN(10);
        tau_vrn4->setDelay(3);
        tau_vrn4->setInput(pcpgc2->getOutput(1));

        addSubnet(tau_vrn3);
        addSubnet(tau_vrn4);
    }
    /*******************************************************************************
	*  MODULE Vrn1,2,3,4 // smooth signal
	*******************************************************************************/
    if (is_decoupled){
    //     cout << "set up Vrn" << endl;
        vrn1  = new VRN();
        vrn3  = new VRN();

        // pcpg to VRN
        w(vrn1->getNeuron(0),  pcpgc1->getNeuron(0), 1.75);
        w(vrn3->getNeuron(0),  tau_vrn3->getNeuron(0), 1.75);

        //
        w(vrn1->getNeuron(1),  vrnInputNeurons[0], 1.0);
        w(vrn3->getNeuron(1),  vrnInputNeurons[2], 1.0);
    //
        addSubnet(vrn1);
        addSubnet(vrn3);
    }
    else{  
        vrn1  = new VRN();
        vrn2  = new VRN();
        vrn3  = new VRN();
        vrn4  = new VRN();

        //        // pcpg to VRN
        w(vrn1->getNeuron(0),  pcpgc1->getNeuron(0), 1.75);
        w(vrn2->getNeuron(0),  pcpgc1->getNeuron(1), 1.75);
        w(vrn3->getNeuron(0),  tau_vrn3->getNeuron(0), 1.75);
        w(vrn4->getNeuron(0),  tau_vrn4->getNeuron(0), 1.75);

        //
        w(vrn1->getNeuron(1),  vrnInputNeurons[0], 1.0);
        w(vrn2->getNeuron(1),  vrnInputNeurons[1], 1.0);
        w(vrn3->getNeuron(1),  vrnInputNeurons[2], 1.0);
        w(vrn4->getNeuron(1),  vrnInputNeurons[3], 1.0);
    //
        addSubnet(vrn1);
        addSubnet(vrn2);
        addSubnet(vrn3);
        addSubnet(vrn4);
    }


    /*******************************************************************************
	*  MODULE Psn0,1,2,3,4,5 // reverse signal
	*******************************************************************************/

    if (is_decoupled){
        psn0 = new PSN();

        /// Psn L1
        //input
        psnInputNeurons[0]->setInput(0);

        w(psn0->getNeuron(0), psnInputNeurons[0], 1);
        w(psn0->getNeuron(1), psnInputNeurons[0], -1);
        // signal
        // w(psn0->getNeuron(2), vrn1->getNeuron(6), 0.5);
        // w(psn0->getNeuron(3), vrn3->getNeuron(6), -0.5);
        // w(psn0->getNeuron(4), vrn3->getNeuron(6), 0.5);
        // w(psn0->getNeuron(5), vrn1->getNeuron(6), -0.5);

        w(psn0->getNeuron(2), pcpgc1->getNeuron(0), 0.5);
        w(psn0->getNeuron(3), tau_vrn3->getNeuron(0), -0.5);
        w(psn0->getNeuron(4), tau_vrn3->getNeuron(0), 0.5);
        w(psn0->getNeuron(5), pcpgc1->getNeuron(0), -0.5);

        addSubnet(psn0);
    }
    else{
        psn0 = new PSN();
        psn1 = new PSN();
        psn2 = new PSN();
        psn3 = new PSN();
        psn4 = new PSN();
        psn5 = new PSN();

        /// Psn L1
        //input
        psnInputNeurons[0]->setInput(0);

        w(psn0->getNeuron(0), psnInputNeurons[0], 1);
        w(psn0->getNeuron(1), psnInputNeurons[0], -1);
        // signal
        w(psn0->getNeuron(2), vrn2->getNeuron(6), 0.5);
        w(psn0->getNeuron(3), vrn4->getNeuron(6), -0.5);
        w(psn0->getNeuron(4), vrn4->getNeuron(6), 0.5);
        w(psn0->getNeuron(5), vrn2->getNeuron(6), -0.5);
        //
        /// Psn L2
        //input
        psnInputNeurons[1]->setInput(0);

        w(psn1->getNeuron(0), psnInputNeurons[1], 1);
        w(psn1->getNeuron(1), psnInputNeurons[1], -1);
        // signal
        w(psn1->getNeuron(2), vrn3->getNeuron(6), 0.5);
        w(psn1->getNeuron(3), vrn1->getNeuron(6), -0.5);
        w(psn1->getNeuron(4), vrn1->getNeuron(6), 0.5);
        w(psn1->getNeuron(5), vrn3->getNeuron(6), -0.5);
        //
        /// Psn L3
        //input
        psnInputNeurons[2]->setInput(0);

        w(psn2->getNeuron(0), psnInputNeurons[2], 1);
        w(psn2->getNeuron(1), psnInputNeurons[2], -1);
        // signal
        w(psn2->getNeuron(2), vrn4->getNeuron(6), 0.5);
        w(psn2->getNeuron(3), vrn2->getNeuron(6), -0.5);
        w(psn2->getNeuron(4), vrn2->getNeuron(6), 0.5);
        w(psn2->getNeuron(5), vrn4->getNeuron(6), -0.5);
        //
        /// Psn R1
        //input
        psnInputNeurons[3]->setInput(0);

        w(psn3->getNeuron(0), psnInputNeurons[3], 1);
        w(psn3->getNeuron(1), psnInputNeurons[3], -1);
        // signal
        w(psn3->getNeuron(2), vrn1->getNeuron(6), 0.5);
        w(psn3->getNeuron(3), vrn3->getNeuron(6), -0.5);
        w(psn3->getNeuron(4), vrn3->getNeuron(6), 0.5);
        w(psn3->getNeuron(5), vrn1->getNeuron(6), -0.5);
        //
        /// Psn R2
        //input
        psnInputNeurons[4]->setInput(0);

        w(psn4->getNeuron(0), psnInputNeurons[4], 1);
        w(psn4->getNeuron(1), psnInputNeurons[4], -1);
        // signal
        w(psn4->getNeuron(2), vrn4->getNeuron(6), 0.5);
        w(psn4->getNeuron(3), vrn2->getNeuron(6), -0.5);
        w(psn4->getNeuron(4), vrn2->getNeuron(6), 0.5);
        w(psn4->getNeuron(5), vrn4->getNeuron(6), -0.5);
        //
        /// Psn R3
        //input
        psnInputNeurons[5]->setInput(0);

        w(psn5->getNeuron(0), psnInputNeurons[5], 1);
        w(psn5->getNeuron(1), psnInputNeurons[5], -1);
        // signal
        w(psn5->getNeuron(2), vrn3->getNeuron(6), 0.5);
        w(psn5->getNeuron(3), vrn1->getNeuron(6), -0.5);
        w(psn5->getNeuron(4), vrn1->getNeuron(6), 0.5);
        w(psn5->getNeuron(5), vrn3->getNeuron(6), -0.5);

        addSubnet(psn0);
        addSubnet(psn1);
        addSubnet(psn2);
        addSubnet(psn3);
        addSubnet(psn4);
        addSubnet(psn5);
    }

    /*******************************************************************************
	*  MODULE pmn // premotor neuron
	*******************************************************************************/
    if (is_decoupled){
        pmn = new PMN();
        w(pmn->getNeuron(0),  psn0->getNeuron(10),  1.0);
        w(pmn->getNeuron(1),  psn0->getNeuron(11),  1.0);

        addSubnet(pmn);
    }
    else{

        pmn = new PMN();

    //    if(cpg_option == 1){
    //		w(pmn->getNeuron(0),  vrn1->getNeuron(6),  1.0);
        w(pmn->getNeuron(0),  psn0->getNeuron(10),  1.0);
        w(pmn->getNeuron(1),  psn0->getNeuron(11),  1.0);
    //		w(pmn->getNeuron(2),  vrn3->getNeuron(6),  1.0);
        w(pmn->getNeuron(2),  psn1->getNeuron(10),  1.0);
        w(pmn->getNeuron(3),  psn1->getNeuron(11),  1.0);
    //		w(pmn->getNeuron(3),  vrn4->getNeuron(6),  1.0);

        w(pmn->getNeuron(4),  psn2->getNeuron(10),  1.0);
        w(pmn->getNeuron(5),  psn2->getNeuron(11),  1.0);

        w(pmn->getNeuron(6),  psn3->getNeuron(10),  1.0);
        w(pmn->getNeuron(7),  psn3->getNeuron(11),  1.0);

        w(pmn->getNeuron(8),  psn4->getNeuron(10),  1.0);
        w(pmn->getNeuron(9),  psn4->getNeuron(11),  1.0);

        w(pmn->getNeuron(10),  psn5->getNeuron(10),  1.0);
        w(pmn->getNeuron(11),  psn5->getNeuron(11),  1.0);

        addSubnet(pmn);
    }

};

void modularController::setPerturbation(double value)
{
    cpg_s->setPerturbation(value);
}

void modularController::setPhii(double value)
{
    if(cpg_option == 2)
        cpg_s->setPhi(value);
    else
        cpg->setPhi(value);
}

void modularController::setMI(double value)
{
    MI = value;
    cpg->setWeight(0, 1, 0.18 + MI);
    cpg->setWeight(1, 0,-0.18 - MI);
}

void modularController::setInputNeuronInput(int input, double value)
{
    setInput(inputNeurons[input],value);
}

double modularController::getFinalNeuronOutput(int output)
{
    return pmn->getOutput(output);
}

double modularController::getCpgOutput(int output)
{
    if(cpg_option == 2)
        return cpg_s->getOutput(output);
    else
        return cpg->getOutput(output);
}

double modularController::getCpgActivity(int output)
{
    if(cpg_option == 2)
        return cpg_s->getActivity(output);
    else
        return cpg->getActivity(output);
}

double modularController::getCpgWeight(int neuron1, int neuron2)
{
    if(cpg_option == 2)
        return cpg_s->getWeight(neuron1, neuron2);
    else
        return cpg->getWeight(neuron1, neuron2);

}

double modularController::getCpgBias(int neuron)
{
    if(cpg_option == 2)
        return cpg_s->getBias(neuron);
    else
        return cpg->getBias(neuron);
}

// Set the output value of a specific CPG neuron
void modularController::setCpgOutput(int neuron, double value)
{
    cpg->setOutput(neuron, value);
}

double modularController::getOutputtau_vrn3(int output)
{
    return tau_vrn3->getOutput(output);
}

double modularController::getpcpgOutput1(int output)
{
    return pcpgc1->getOutput(output);
}

double modularController::getpcpgOutput2(int output)
{
    return pcpgc2->getOutput(output);
}

void modularController::setCpgInputNeuron(double value)
{
    cpgInputNeuron[0]->setInput(value*cos(cpg->getOutput(0)));
    cpgInputNeuron[1]->setInput(value*sin(cpg->getOutput(1)));
}

void modularController::setInputPsn(int input, double value)
{
    psn->setInput(input,value);
}

void modularController::setInputVrnLeft(int input, double value)
{
    vrnLeft->setInput(input,value);
}

void modularController::setInputVrnRight(int input, double  value)
{
    vrnRight->setInput(input,value);
}

double modularController::getPsnOutput(int output)
{
    return psn->getOutput(output);
}

double modularController::getVrnLeftOutput(int output)
{
    return vrnLeft->getOutput(output);
}

double modularController::getVrnRightOutput(int output)
{
    return vrnRight->getOutput(output);
}

double modularController::getMI()
{
    return MI;
}

double modularController::getPhi()
{
    if(cpg_option == 2)
        return cpg_s->getPhi();
    else
        return cpg->getPhi();
}

void modularController::step()
{
    if (isDecoupled){
        tau_vrn3->setInput(pcpgc2->getOutput(0));
    }
    else{
        tau_vrn3->setInput(pcpgc2->getOutput(0));
    //    std::cout << "pcpg2 : " << pcpgc2->getOutput(0) << std::endl;
    //    std::cout << "tau_vrn3 : " << vrn3->getOutput(0) << std::endl;
        tau_vrn4->setInput(pcpgc2->getOutput(1));
    }
    updateActivities();
    // updateWeights();
    updateOutputs();
    postProcessing();

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// Rectification of one half of the CPG signal
// direction = 1 -> All values lower than lim will be equal to lim
// direction = -1 -> All values greater than lim will be equal to lim
float modularController::truncateSignal(float value, float lim, int direction)
{
    float result;
    if(direction == 1)
    {
        if(value >= lim)
        {
            result = value;
        }
        else
        {
            result = lim;
        }
    }
    else
    {
        if(value >= lim)
        {
            result = lim;
        }
        else
        {
            result = value;
        }
    }

    return result;
}



float modularController::trimSignal(float value, float prev, float min, int direction)
{
    float result;
    if(direction == 1)
    {
        if(value >= min)
        {
            float diff = value - prev;
            if(diff >= 0)
            {
                result = value;
            }
            else
            {
                result = min;
            }

        }
        else
        {
            result = value;
        }
    }
    else
    {
        if(value > min)
        {
            result = value;
        }
        else
        {
            float diff = value - prev;
            if(diff >= 0)
            {
                result = min;
            }
            else
            {
                result = value;
            }
        }
    }


    return result;
}

void modularController::setPsnInputNeurons(int input, double value)
{
    setInput(psnInputNeurons[input],value);

}

void modularController::setVrnInputNeurons(int input, double value)
{
    setInput(vrnInputNeurons[input],value);

}

double modularController::getVrnOutput1(int output)
{
    return vrn1->getOutput(output);
}

double modularController::getVrnOutput2(int output)
{
    return vrn2->getOutput(output);
}

double modularController::getVrnOutput3(int output)
{
    return vrn3->getOutput(output);
}

double modularController::getVrnOutput4(int output)
{
    return vrn4->getOutput(output);
}

double modularController::getPsnOutput0(int output)
{
    return psn0->getOutput(output);
}

double modularController::getpmnOutput(int output)
{
    return pmn->getOutput(output);
}

