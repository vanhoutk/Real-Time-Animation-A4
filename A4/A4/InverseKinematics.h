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
#include "Bone.h"

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
	theta2 = degrees(2 * atanf(sqrt(fractionB)));
}

#define MAX_IK_TRIES 500
#define IK_POS_THRESH 0.1f

bool CCDIK(vec3 position, int endEffectorIndex, int chainRootIndex, Bone** bones)
{
	vec3		rootPos, curEnd, desiredEnd, targetVector, curVector, rotationAxis;
	GLfloat		cosAngle, turnAngle;
	int			bone_index, tries;

	bone_index = endEffectorIndex - 1;
	tries = 0;

	float distance = sqrt(get_squared_dist(position, bones[chainRootIndex]->getPosition()));
	float maxReach = 0;

	for (int i = bone_index; i >= chainRootIndex; i--)
	{
		maxReach += sqrt(get_squared_dist(bones[i + 1]->getPosition(), bones[i]->getPosition()));
	}

	if (distance > maxReach)
	{
		cout << "Distance > MaxReach" << endl;
		vec3 endDirection = position - bones[chainRootIndex]->getPosition();
		endDirection = endDirection / distance;
		endDirection = endDirection * maxReach;
		position = endDirection + bones[chainRootIndex]->getPosition();
	}

	do
	{
		rootPos = bones[bone_index]->getPosition();
		curEnd = bones[endEffectorIndex]->getPosition(); // Position of the end effector
		desiredEnd = position; // Desired end effector position

		if (get_squared_dist(curEnd, desiredEnd) > IK_POS_THRESH)
		{
			curVector = curEnd - rootPos;
			targetVector = position - rootPos;

			curVector = normalise(curVector);
			targetVector = normalise(targetVector);

			cosAngle = dot(targetVector, curVector);

			//if (cosAngle < 0.99999)
			//{
				rotationAxis = cross(curVector, targetVector);
				rotationAxis = normalise(rotationAxis);
				turnAngle = acos((float)cosAngle);
				if(turnAngle > 0.1f * ONE_DEG_IN_RAD)
					bones[bone_index]->rotateJoint(turnAngle, rotationAxis);
			//}

			if (--bone_index < chainRootIndex)
				bone_index = endEffectorIndex - 1;	// Go back to the start of the chain
		}
	} while (tries++ < MAX_IK_TRIES && get_squared_dist(curEnd, desiredEnd) > IK_POS_THRESH);

	if (tries >= MAX_IK_TRIES)
		return false;
	else
		return true;
}

vec3 splinePositionBezier(vec3 p1, vec3 p2, vec3 p3, vec3 p4, float t)
{
	vec3 term1 = p1 * pow(1 - t, 3);
	vec3 term2 = p2 * 3 * t * pow(1 - t, 2);
	vec3 term3 = p3 * 3 * t * t * (1 - t);
	vec3 term4 = p4 * pow(t, 3);
	return term1 + term2 + term3 + term4;
}

vec3 splinePositionCatmullRom(vec3 p1, vec3 p2, vec3 p3, vec3 p4, float t)
{
	vec3 term1 = p1;
	vec3 term2 = (p3 - p1) * t * 0.5;
	vec3 term3 = ((p1 * 2) - (p2 * 5) + (p3 * 4) - p4) * t * t * 0.5;
	vec3 term4 = ((p1 * -1) + (p2 * 3) - (p3 * 3) + p4) * t * t * t * 0.5;
	return term1 + term2 + term3 + term4;
}

float GetT(float t, vec3 p0, vec3 p1)
{
	float alpha = 0.5f;
	float a = pow((p1.v[0] - p0.v[0]), 2.0f) + pow((p1.v[1] - p0.v[1]), 2.0f) + pow((p1.v[2] - p0.v[2]), 2.0f);
	float b = pow(a, 0.5f);
	float c = pow(b, alpha);

	return (c + t);
}

vec3 splinePositionCentripetalCatmullRom(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float t)
{
	float t0 = 0.0f;
	float t1 = GetT(t0, p0, p1);
	float t2 = GetT(t1, p1, p2);
	float t3 = GetT(t2, p2, p3);

	//for (float t = t1; t<t2; t += ((t2 - t1) / 2))
	//{
		vec3 A1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
		vec3 A2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
		vec3 A3 = p3 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));

		vec3 B1 = A1 * ((t2 - t) / (t2 - t0)) + A2 * ((t - t0) / (t2 - t0));
		vec3 B2 = A2 * ((t3 - t) / (t3 - t1)) + A3 * ((t - t1) / (t3 - t1));

		vec3 C = B1 * ((t2 - t) / (t2 - t1)) + B2 * ((t - t1) / (t2 - t1));

		return C;
	//}
}

