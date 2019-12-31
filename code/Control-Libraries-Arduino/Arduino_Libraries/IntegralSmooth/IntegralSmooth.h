/*
 * IntegralSmooth.h
 *
 *  Created on: Jun 3, 2015
 *      Author: adrien
 */

#ifndef INTEGRALSMOOTH_H_
#define INTEGRALSMOOTH_H_

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }


class IntegralSmooth
{
private:
	int updateFreqHz;
	float maxIter;
	int nbIterSameSign;
	float output;
	float alphaSmooth;
	bool lastValPositive;
public:
	IntegralSmooth();
	IntegralSmooth(float pAlphaSmooth, int pUpdateFreqHz) ;

	void init(float pAlphaSmooth, int pUpdateFreqHz);

	void update(double e, double dt);

	float getOutput();
};


#endif /* MATH_INTEGRALSMOOTH_H_ */
