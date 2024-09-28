/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/


#include "grabcpg.h"

#include <cmath>
#include <sstream>

GRABCPG::GRABCPG()
{
    setDefaultTransferFunction(tanhFunction());
    setNeuronNumber(2);

    alpha = 1.01;
    phi   = 0.1*M_PI;
    updateSO2Weights();
    frequencyTableEnabled = false;
    updateFrequencyTable();
}

void GRABCPG::enableFrequencyTable(const bool enabled)
{
    if (enabled == frequencyTableEnabled) return;
    frequencyTableEnabled = enabled;
    updateFrequencyTable();
}

const double& GRABCPG::getAlpha() const
{
    return alpha;
}

const double GRABCPG::getFrequency() const
{
    if (frequencyTableEnabled)
        return frequencyTable.y(phi/M_PI);
    else
        //linear approximation
        return (0.5/M_PI)*phi;
}

const double& GRABCPG::getPhi() const
{
    return phi;
}

const double GRABCPG::getPhi(const double & afrequency) const
{
    if (frequencyTableEnabled)
        return frequencyTable.x(afrequency)*M_PI;
    else
        // linear approximation
        return 2*afrequency*M_PI;
}

void GRABCPG::setAlpha(const double& aalpha)
{
    alpha = aalpha;
    updateSO2Weights();
    updateFrequencyTable();
}

void GRABCPG::setFrequency(const double& afrequency)
{
    setPhi(getPhi(afrequency));
}

void GRABCPG::setPhi(const double& aphi)
{
    phi = aphi;
    updateSO2Weights();
}

void GRABCPG::setGamma(const double& agamma)
{
    gamma = agamma;
}

void GRABCPG::updateLoss(float targetPose1, float currentPose1, float targetPose2, float currentPose2)
{
    loss[0] = targetPose1 - currentPose1;
    loss[1] = targetPose2 - currentPose2;
    output[0] = getOutput(0) - gamma * loss[0];
    output[1] = getOutput(1) - gamma * loss[1];
    setOutput(0, output[0]);
    setOutput(1, output[1]);
}

void GRABCPG::updateFrequencyTable()
{
    if (!frequencyTableEnabled) return;
    std::stringstream filename;
    filename << "grabcpg_fVsPhi_a" << alpha << ".dat";
    frequencyTable.load(filename.str().c_str());
}

void GRABCPG::updateSO2Weights()
{
    w(0, 0,  alpha * cos(phi));
    w(0, 1,  alpha * sin(phi));
    w(1, 0, -alpha * sin(phi));
    w(1, 1,  alpha * cos(phi));
}

void GRABCPG::postProcessing()
{

}
