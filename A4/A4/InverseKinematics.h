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
	//theta2 = degrees(acosf(fraction2));

	theta2 = degrees(2 * atanf(sqrt(fractionB)));


}

/*#define MAX_IK_TRIES 100 // TIMES THROUGH THE CCD LOOP
#define IK_POS_THRESH 0.1f // THRESHOLD FOR SUCCESS

///////////////////////////////////////////////////////////////////////////////
// Procedure:	ComputeCCDLink
// Purpose:		Compute an IK Solution to an end effector position in 3D
// Arguments:	End Target (x,y,z)
// Returns:		TRUE if a solution exists, FALSE if the position isn't in reach
///////////////////////////////////////////////////////////////////////////////	
bool CCDIK(vec3 position, Bone* bones, int endEffectorIndex, int chainRootIndex)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	vec3		rootPos, curEnd, desiredEnd, targetVector, curVector, crossResult;
	GLfloat		cosAngle, turnAngle, turnDeg;
	int			bone_index, tries;
	versor		aquat;
	///////////////////////////////////////////////////////////////////////////////
	// START AT THE LAST bone_index IN THE CHAIN
	bone_index = endEffectorIndex - 1;
	tries = 0;						// LOOP COUNTER SO I KNOW WHEN TO QUIT
	do
	{
		// THE COORDS OF THE X,Y,Z POSITION OF THE ROOT OF THIS BONE IS IN THE MATRIX
		// TRANSLATION PART WHICH IS IN THE 12,13,14 POSITION OF THE MATRIX
		rootPos.v[0] = bones[bone_index].getGlobalTransformation.m[12];
		rootPos.v[1] = bones[bone_index].getGlobalTransformation.m[13];
		rootPos.v[2] = bones[bone_index].getGlobalTransformation.m[14];

		// POSITION OF THE END EFFECTOR
		curEnd.v[0] = bones[endEffectorIndex].getGlobalTransformation.m[12];
		curEnd.v[1] = bones[endEffectorIndex].getGlobalTransformation.m[13];
		curEnd.v[2] = bones[endEffectorIndex].getGlobalTransformation.m[14];

		// DESIRED END EFFECTOR POSITION
		desiredEnd.v[0] = position.v[0];
		desiredEnd.v[1] = position.v[1];
		desiredEnd.v[2] = position.v[2];

		// SEE IF I AM ALREADY CLOSE ENOUGH
		if (get_squared_dist(curEnd, desiredEnd) > IK_POS_THRESH)
		{
			// CREATE THE VECTOR TO THE CURRENT EFFECTOR POS
			curVector.v[0] = curEnd.v[0] - rootPos.v[0];
			curVector.v[1] = curEnd.v[1] - rootPos.v[1];
			curVector.v[2] = curEnd.v[2] - rootPos.v[2];

			// CREATE THE DESIRED EFFECTOR POSITION VECTOR
			targetVector.v[0] = position.v[0] - rootPos.v[0];
			targetVector.v[1] = position.v[1] - rootPos.v[1];
			targetVector.v[2] = position.v[2] - rootPos.v[2];

			// NORMALIZE THE VECTORS (EXPENSIVE, REQUIRES A SQRT)
			curVector = normalise(curVector);
			targetVector = normalise(targetVector);

			// THE DOT PRODUCT GIVES ME THE COSINE OF THE DESIRED ANGLE
			cosAngle = dot(targetVector, curVector);

			// IF THE DOT PRODUCT RETURNS 1.0, I DON'T NEED TO ROTATE AS IT IS 0 DEGREES
			if (cosAngle < 0.99999)
			{
				// USE THE CROSS PRODUCT TO CHECK WHICH WAY TO ROTATE
				crossResult = cross(curVector, targetVector);
				crossResult = normalise(crossResult);
				turnAngle = acos((float)cosAngle);	// GET THE ANGLE
				//turnDeg = degrees(turnAngle);		// COVERT TO DEGREES
													// DAMPING
				//if (m_Damping && turnDeg > bones[bone_index].damp_width)
				//	turnDeg = bones[bone_index].damp_width;
				
				//AxisAngleToQuat(&crossResult, turnDeg, &aquat);
				//MultQuaternions(&bones[bone_index].quat, &aquat, &bones[bone_index].quat);

				bones[bone_index].rotateJoint(turnAngle, crossResult);
				
				// HANDLE THE DOF RESTRICTIONS IF I WANT THEM
				//if (m_DOF_Restrict)
				//	CheckDOFRestrictions(&bones[bone_index]);
				
				// RECALC ALL THE MATRICES WITHOUT DRAWING ANYTHING
				//drawScene(FALSE);		// CHANGE THIS TO TRUE IF YOU WANT TO SEE THE ITERATION
			}
			if (--bone_index < 0) bone_index = endEffectorIndex - 1;	// START OF THE CHAIN, RESTART
		}
		// QUIT IF I AM CLOSE ENOUGH OR BEEN RUNNING LONG ENOUGH
	} while (tries++ < MAX_IK_TRIES &&
		get_squared_dist(curEnd, desiredEnd) > IK_POS_THRESH);
	if (tries == MAX_IK_TRIES)
		return FALSE;
	else
		return TRUE;
}*/