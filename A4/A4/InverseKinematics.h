#pragma once

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <math.h>
#include <mmsystem.h>
#include <stdio.h>
#include <vector>				// STL dynamic memory
#include <windows.h>

#include "Antons_maths_funcs.h" // Anton's maths functions

void analyticalIK(vec3 endPosition, float L1, float L2, float& theta1, float& theta2)
{
	float x = endPosition.v[0];
	float y = endPosition.v[1];
	float z = endPosition.v[2];

	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;

	float distance = sqrt(x2 + y2 + z2);

	if (distance > (L1 + L2 - 0.01f))
	{
		vec3 newEndPosition = (endPosition / distance) * (L1 + L2 - 0.01f);
		x = newEndPosition.v[0];
		y = newEndPosition.v[1];
		z = newEndPosition.v[2];

		x2 = x * x;
		y2 = y * y;
		z2 = z * z;

		distance = sqrt(x2 + y2 + z2);
	}


	float L12 = L1 * L1;
	float L22 = L2 * L2;

	float distance2 = x2 + y2 + z2;

	float distanceXZ = sqrt(x2 + z2);
	float distanceXY = sqrt(x2 + y2);
	float distanceYZ = sqrt(y2 + z2);

	float thetaT = 180 - degrees(acos(x / distance));
	
	if (y >= 0.0f)
		thetaT *= -1.0f;

	float fraction1 = (L12 + distance2 - L22) / (2 * L1 * distance);
	float fraction2 = (L12 + L22 - distance2) / (2 * L1 * L2);

	float fractionB = (pow((L1 + L2), 2) - distance2) / (distance2 - pow((L1 - L2), 2));

	theta1 = thetaT - degrees(acosf(fraction1));
	//theta2 = degrees(acosf(fraction2));

	theta2 = degrees(2 * atanf(sqrt(fractionB)));


}