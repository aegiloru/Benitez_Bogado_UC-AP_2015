#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "IntegralSmooth.h"
#include <math.h>
	IntegralSmooth::IntegralSmooth() {
		// Initialize with default value
		// Alpha = 0.98, Update freq Hz = 50 Hz
		init(0.98, 50);
	}
	IntegralSmooth::IntegralSmooth(float pAlphaSmooth, int pUpdateFreqHz) {
		init(pAlphaSmooth, pUpdateFreqHz);
	}
	void IntegralSmooth::init(float pAlphaSmooth, int pUpdateFreqHz)
	{
		alphaSmooth = pAlphaSmooth;
		updateFreqHz = pUpdateFreqHz;
		maxIter = (float) pUpdateFreqHz;
		nbIterSameSign = 0;
		output = 0.0;
		lastValPositive = true;
	}
	void IntegralSmooth::update(double e, double dt)
	{
		output = alphaSmooth * output + (nbIterSameSign / maxIter) * e * dt;

		if (output >= 0.0)
		{
			if (lastValPositive) {
				nbIterSameSign ++;
			}
			else {
				nbIterSameSign = 1;
			}
			lastValPositive = true;
		}
		else {
			if (lastValPositive == false) {
				nbIterSameSign ++;
			}
			else {
				nbIterSameSign = 1;
			}

			lastValPositive = false;
		}

		Bound(nbIterSameSign, 0, maxIter);
	}
		float IntegralSmooth::getOutput()
	{
		return output;
	}
	