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

	float L12 = L1 * L1;
	float L22 = L2 * L2;

	// d
	float distance = sqrt(x2 + y2 + z2);

	float distanceXZ = sqrt(x2 + z2);
	float distanceXY = sqrt(x2 + y2);
	float distanceYZ = sqrt(y2 + z2);

	float thetaT = acosf(distanceXZ / distance);

	theta1 = acosf((L12 + x2 + y2 + x2 - L22) / (2 * L1 * distance)) + thetaT;
	theta2 = acosf((L12 + L22 - (x2 + y2 + z2)) / (2 * L1 * L2));
}