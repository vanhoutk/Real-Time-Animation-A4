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
#include "InverseKinematics.h"
#include "Mesh.h"

class Skeleton {
public:
	Bone* rootBone;
	Bone* bones[20];
	int numBones = 20;
	int handIndex = 4;

	Skeleton();
	void createHand(Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell);
	void createTorso(Mesh torsoMesh, Mesh upperArmShell, Mesh lowerArmShell, Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell);
	void drawSkeleton(mat4 view, mat4 projection, vec4 viewPosition);
	void rotateWrist360();
	void closeFist();
	void openFist();
	void closeAndOpenFist();
	void moveTo(vec3 position);
	void moveToCCD(vec3 position);
	bool CCDIK(vec3 position, int endEffectorIndex, int chainRootIndex);
private:
	GLfloat count = 0.0f;
	GLfloat speed = 0.15f;
	bool close = true;
	bool once = false;
	GLfloat theta1 = 0.0f;
	GLfloat theta2 = 0.0f;
};

Skeleton::Skeleton()
{

}

void Skeleton::createHand(Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell)
{
	vec4 joint_colour = vec4(1.0f, 1.0f, 0.0f, 1.0f);
	vec4 shell_colour = vec4(1.0f, 1.0f, 1.0f, 1.0f);

	bones[handIndex+0] = new Bone("hand", true, scale(identity_mat4(), vec3(0.2f, 0.2f, 0.2f)), handMesh, handShell, true, joint_colour, shell_colour);
	rootBone = bones[handIndex+0];

	// Thumb
	mat4 thumb1_local = scale(identity_mat4(), vec3(0.75f, 1.0f, 1.0f) * 0.8f);
	thumb1_local = translate(thumb1_local, vec3(3.0f, 0.0f, 2.5f));
	bones[handIndex+1] = new Bone("thumb_1", bones[handIndex+0], thumb1_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+1]);
	bones[handIndex+1]->pivotJoint(radians(30.0f));
	bones[handIndex+1]->rollJoint(radians(45.0f));
	
	mat4 thumb2_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+2] = new Bone("thumb_2", bones[handIndex+1], thumb2_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+1]->addChild(bones[handIndex+2]);
	bones[handIndex+2]->bendJoint(radians(2.0f));
	
	mat4 thumb3_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+3] = new Bone("thumb_3", bones[handIndex+2], thumb3_local, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+2]->addChild(bones[handIndex+3]);
	bones[handIndex+3]->bendJoint(radians(5.0f));

	// Finger 1
	mat4 finger11_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger11_local = translate(finger11_local, vec3(-4.0f, 0.0f, 3.1f));
	bones[handIndex+4] = new Bone("finger_11", bones[handIndex+0], finger11_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+4]);

	mat4 finger_joint_offset = scale(identity_mat4(), vec3(0.85f, 0.85f, 0.85f));
	finger_joint_offset = translate(finger_joint_offset, vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+5] = new Bone("finger_12", bones[handIndex+4], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+4]->addChild(bones[handIndex+5]);

	mat4 finger_tip_offset = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+6] = new Bone("finger_13", bones[handIndex+5], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+5]->addChild(bones[handIndex+6]);

	// Finger 2
	mat4 finger21_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f) * 0.8f);
	finger21_local = translate(finger21_local, vec3(-4.0f, 0.0f, 0.9f));
	bones[handIndex+7] = new Bone("finger_21", bones[handIndex+0], finger21_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+7]);

	bones[handIndex+8] = new Bone("finger_22", bones[handIndex+7], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+7]->addChild(bones[handIndex+8]);

	bones[handIndex+9] = new Bone("finger_23", bones[handIndex+8], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+8]->addChild(bones[handIndex+9]);

	// Finger 3
	mat4 finger31_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger31_local = translate(finger31_local, vec3(-4.0f, 0.0f, -1.3f));
	bones[handIndex+10] = new Bone("finger_31", bones[handIndex+0], finger31_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+10]);

	bones[handIndex+11] = new Bone("finger_32", bones[handIndex+10], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+10]->addChild(bones[handIndex+11]);

	bones[handIndex+12] = new Bone("finger_33", bones[handIndex+11], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+11]->addChild(bones[handIndex+12]);

	// Finger 4
	mat4 finger41_local = scale(identity_mat4(), vec3(0.7f, 0.7f, 0.7f) * 0.8f);
	finger41_local = translate(finger41_local, vec3(-4.0f, 0.0f, -3.1f));
	bones[handIndex+13] = new Bone("finger_41", bones[handIndex+0], finger41_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+13]);

	bones[handIndex+14] = new Bone("finger_42", bones[handIndex+13], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+13]->addChild(bones[handIndex+14]);

	bones[handIndex+15] = new Bone("finger_43", bones[handIndex+14], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+14]->addChild(bones[handIndex+15]);
}

void Skeleton::createTorso(Mesh torsoMesh, Mesh upperArmShell, Mesh lowerArmShell, Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell)
{
	vec4 joint_colour = vec4(1.0f, 1.0f, 0.0f, 1.0f);
	vec4 shell_colour = vec4(1.0f, 1.0f, 1.0f, 1.0f);

	// Torso
	mat4 torso_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f));
	//torso_local = translate(torso_local, vec3(-5.0f, 0.0f, 0.0f));
	bones[0] = new Bone("torso", true, torso_local, torsoMesh, Mesh(), false, joint_colour);
	rootBone = bones[0];

	// Shoulders
	mat4 shoulder_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f));
	shoulder_local = translate(shoulder_local, vec3(2.5f, 7.5f, 0.0f));
	bones[1] = new Bone("shoulders", bones[0], shoulder_local, jointMesh, Mesh(), false, joint_colour);
	bones[0]->addChild(bones[1]);

	// Upper Arm
	mat4 upper_arm_local = scale(identity_mat4(), vec3(1.2f, 1.2f, 1.2f) * 0.8f);
	upper_arm_local = translate(upper_arm_local, vec3(-5.0f, 0.0f, 0.0f));
	bones[2] = new Bone("upper_arm", bones[1], upper_arm_local, jointMesh, upperArmShell, true, joint_colour, shell_colour);
	bones[1]->addChild(bones[2]);
	bones[2]->rollJoint(radians(-90.0f));
	//bones[2]->pivotJoint(radians(-70.0f));

	// Lower Arm
	mat4 lower_arm_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f));
	lower_arm_local = translate(lower_arm_local, vec3(-5.0f, 0.0f, 0.0f));
	bones[3] = new Bone("lower_arm", bones[2], lower_arm_local, jointMesh, lowerArmShell, true, joint_colour, shell_colour);
	bones[2]->addChild(bones[3]);
	//bones[3]->pivotJoint(radians(-20.0f));

	// Hand
	mat4 hand_local = scale(identity_mat4(), vec3(0.3f, 0.3f, 0.3f));
	hand_local = translate(hand_local, vec3(-7.0f, 0.0f, 0.0f));
	bones[handIndex+0] = new Bone("hand", bones[3], hand_local, handMesh, handShell, true, joint_colour, shell_colour);
	bones[3]->addChild(bones[handIndex+0]);
	bones[handIndex+0]->rollJoint(radians(90.0f));

	// Thumb
	mat4 thumb1_local = scale(identity_mat4(), vec3(0.75f, 1.0f, 1.0f) * 0.8f);
	thumb1_local = translate(thumb1_local, vec3(3.0f, 0.0f, 2.5f));
	bones[handIndex+1] = new Bone("thumb_1", bones[handIndex+0], thumb1_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+1]);
	bones[handIndex+1]->pivotJoint(radians(30.0f));
	bones[handIndex+1]->rollJoint(radians(45.0f));

	mat4 thumb2_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+2] = new Bone("thumb_2", bones[handIndex+1], thumb2_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+1]->addChild(bones[handIndex+2]);
	bones[handIndex+2]->bendJoint(radians(2.0f));

	mat4 thumb3_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+3] = new Bone("thumb_3", bones[handIndex+2], thumb3_local, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+2]->addChild(bones[handIndex+3]);
	bones[handIndex+3]->bendJoint(radians(5.0f));

	// Finger 1
	mat4 finger11_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger11_local = translate(finger11_local, vec3(-4.0f, 0.0f, 3.1f));
	bones[handIndex+4] = new Bone("finger_11", bones[handIndex+0], finger11_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+4]);

	mat4 finger_joint_offset = scale(identity_mat4(), vec3(0.85f, 0.85f, 0.85f));
	finger_joint_offset = translate(finger_joint_offset, vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+5] = new Bone("finger_12", bones[handIndex+4], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+4]->addChild(bones[handIndex+5]);

	mat4 finger_tip_offset = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[handIndex+6] = new Bone("finger_13", bones[handIndex+5], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+5]->addChild(bones[handIndex+6]);

	// Finger 2
	mat4 finger21_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f) * 0.8f);
	finger21_local = translate(finger21_local, vec3(-4.0f, 0.0f, 0.9f));
	bones[handIndex+7] = new Bone("finger_21", bones[handIndex+0], finger21_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+7]);

	bones[handIndex+8] = new Bone("finger_22", bones[handIndex+7], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+7]->addChild(bones[handIndex+8]);

	bones[handIndex+9] = new Bone("finger_23", bones[handIndex+8], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+8]->addChild(bones[handIndex+9]);

	// Finger 3
	mat4 finger31_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger31_local = translate(finger31_local, vec3(-4.0f, 0.0f, -1.3f));
	bones[handIndex+10] = new Bone("finger_31", bones[handIndex+0], finger31_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+10]);

	bones[handIndex+11] = new Bone("finger_32", bones[handIndex+10], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+10]->addChild(bones[handIndex+11]);

	bones[handIndex+12] = new Bone("finger_33", bones[handIndex+11], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+11]->addChild(bones[handIndex+12]);

	// Finger 4
	mat4 finger41_local = scale(identity_mat4(), vec3(0.7f, 0.7f, 0.7f) * 0.8f);
	finger41_local = translate(finger41_local, vec3(-4.0f, 0.0f, -3.1f));
	bones[handIndex+13] = new Bone("finger_41", bones[handIndex+0], finger41_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+0]->addChild(bones[handIndex+13]);

	bones[handIndex+14] = new Bone("finger_42", bones[handIndex+13], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[handIndex+13]->addChild(bones[handIndex+14]);

	bones[handIndex+15] = new Bone("finger_43", bones[handIndex+14], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[handIndex+14]->addChild(bones[handIndex+15]);
}

void Skeleton::drawSkeleton(mat4 view, mat4 projection, vec4 viewPosition = vec4(0.0f, 0.0f, 0.0f, 0.0f))
{
	rootBone->drawBone(view, projection, viewPosition);
}

void Skeleton::rotateWrist360()
{
	bones[handIndex]->rollJoint(radians(speed));
}

void Skeleton::closeFist()
{
	if (close)
	{
		count += speed;
		for (int i = handIndex + 1; i < handIndex + 4; i++)
			bones[i]->bendJoint(radians(speed * (i - handIndex) / 10.0f));
		for (int i = handIndex + 4; i < numBones; i++)
			bones[i]->bendJoint(radians(speed));
	}
	if (count >= 90.0f)
		close = false;
}

void Skeleton::openFist()
{
	if(!close)
	{
		count -= speed;
		for (int i = handIndex + 1; i < handIndex + 4; i++)
			bones[i]->bendJoint(radians(-speed * (i - handIndex) / 10.0f));
		for (int i = handIndex + 4; i < numBones; i++)
			bones[i]->bendJoint(radians(-speed));
	}

	if (count <= 0.0f)
		close = true;
}

void Skeleton::closeAndOpenFist()
{
	closeFist();
	openFist();
}

void Skeleton::moveTo(vec3 position)
{
	//float theta1;
	//float theta2;
	//vec4 start_position = bones[2]->getGlobalTransformation() * vec4(0.0f, 0.0f, 0.0f, 0.0f);
	//vec3 start_position_vec3 = vec3(start_position.v[0], start_position.v[1], start_position.v[2]);

	bones[3]->pivotJoint(1.0f * radians(theta2));
	bones[2]->pivotJoint(1.0f * radians(theta1));

	vec3 start_position = vec3(-2.5f, 7.5f, 0.0f);
	analyticalIK(position - start_position, 5.0f, 5.0f, theta1, theta2);

	//if (true)
	//{
	//	once = !once;
		bones[2]->pivotJoint(-1.0f * radians(theta1));
		bones[3]->pivotJoint(-1.0f * radians(theta2));
	//}
}

void Skeleton::moveToCCD(vec3 position)
{
	CCDIK(position, handIndex, handIndex - 2);
}

///////////////////////////////////////////////////////////////////////////////
// Function:	QuatToEuler
// Purpose:		Convert a Quaternion back to Euler Angles
// Arguments:	Quaternions and target Euler vector
// Notes:		The method is to convert Quaternion to a 3x3 matrix and
//				decompose the matrix.  This is subject to the
//				ambiguities of square roots and problems with inverse trig.
//				Matrix to Euler conversion is really very ill-defined but works
//				for my purposes.
///////////////////////////////////////////////////////////////////////////////
/*void QuatToEuler(const versor *quat, vec3 *euler)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	float matrix[3][3];
	float cx, sx, x;
	float cy, sy, y, yr;
	float cz, sz, z;
	///////////////////////////////////////////////////////////////////////////////
	// CONVERT QUATERNION TO MATRIX - I DON'T REALLY NEED ALL OF IT
	matrix[0][0] = 1.0f - (2.0f * quat->q[1] * quat->q[1]) - (2.0f * quat->q[2] * quat->q[2]); 
	//	matrix[0][1] = (2.0f * quat->x * quat->y) - (2.0f * quat->w * quat->z);
	//	matrix[0][2] = (2.0f * quat->x * quat->z) + (2.0f * quat->w * quat->y);
	matrix[1][0] = (2.0f * quat->q[0] * quat->q[1]) + (2.0f * quat->q[3] * quat->q[2]);
	//	matrix[1][1] = 1.0f - (2.0f * quat->x * quat->x) - (2.0f * quat->z * quat->z);
	//	matrix[1][2] = (2.0f * quat->y * quat->z) - (2.0f * quat->w * quat->x);
	matrix[2][0] = (2.0f * quat->q[0] * quat->q[2]) - (2.0f * quat->q[3] * quat->q[1]);
	matrix[2][1] = (2.0f * quat->q[1] * quat->q[2]) + (2.0f * quat->q[3] * quat->q[0]);
	matrix[2][2] = 1.0f - (2.0f * quat->q[0] * quat->q[0]) - (2.0f * quat->q[1] * quat->q[1]);

	sy = -matrix[2][0];
	cy = sqrt(1 - (sy * sy));
	yr = (float)atan2(sy, cy);
	euler->v[1] = (yr * 180.0f) / (float)M_PI;

	// AVOID DIVIDE BY ZERO ERROR ONLY WHERE Y= +-90 or +-270 
	// NOT CHECKING cy BECAUSE OF PRECISION ERRORS
	if (sy != 1.0f && sy != -1.0f)
	{
		cx = matrix[2][2] / cy;
		sx = matrix[2][1] / cy;
		euler->v[0] = ((float)atan2(sx, cx) * 180.0f) / (float)M_PI;	// RAD TO DEG

		cz = matrix[0][0] / cy;
		sz = matrix[1][0] / cy;
		euler->v[2] = ((float)atan2(sz, cz) * 180.0f) / (float)M_PI;	// RAD TO DEG
	}
	else
	{
		// SINCE Cos(Y) IS 0, I AM SCREWED.  ADOPT THE STANDARD Z = 0
		// I THINK THERE IS A WAY TO FIX THIS BUT I AM NOT SURE.  EULERS SUCK
		// NEED SOME MORE OF THE MATRIX TERMS NOW
		matrix[1][1] = 1.0f - (2.0f * quat->q[0] * quat->q[0]) - (2.0f * quat->q[2] * quat->q[2]);
		matrix[1][2] = (2.0f * quat->q[1] * quat->q[2]) - (2.0f * quat->q[3] * quat->q[0]);
		cx = matrix[1][1];
		sx = -matrix[1][2];
		euler->v[0] = ((float)atan2(sx, cx) * 180.0f) / (float)M_PI;	// RAD TO DEG

		cz = 1.0f;
		sz = 0.0f;
		euler->v[2] = ((float)atan2(sz, cz) * 180.0f) / (float)M_PI;	// RAD TO DEG
	}
}*/
// QuatToEuler  ///////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function:	EulerToQuaternion2
// Purpose:		Convert a set of Euler angles to a Quaternion
// Arguments:	A rotation set of 3 angles, a quaternion to set
// Discussion:  This is a second variation.  It creates a
//				Series of quaternions and multiplies them together
//				It would be easier to extend this for other rotation orders
///////////////////////////////////////////////////////////////////////////////
/*void EulerToQuaternion2(vec3 *rot, versor *quat)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	float rx, ry, rz, ti, tj, tk;
	versor qx, qy, qz, qf;
	///////////////////////////////////////////////////////////////////////////////
	// FIRST STEP, CONVERT ANGLES TO RADIANS
	rx = (rot->v[0] * (float)M_PI) / (360 / 2);
	ry = (rot->v[1] * (float)M_PI) / (360 / 2);
	rz = (rot->v[2] * (float)M_PI) / (360 / 2);
	// GET THE HALF ANGLES
	ti = rx * (float)0.5;
	tj = ry * (float)0.5;
	tk = rz * (float)0.5;

	qx.q[0] = (float)sin(ti); qx.q[1] = 0.0; qx.q[2] = 0.0; qx.q[3] = (float)cos(ti);
	qy.q[0] = 0.0; qy.q[1] = (float)sin(tj); qy.q[2] = 0.0; qy.q[3] = (float)cos(tj);
	qz.q[0] = 0.0; qz.q[1] = 0.0; qz.q[2] = (float)sin(tk); qz.q[3] = (float)cos(tk);

	qf = (qx * qy) * qz;
	//MultQuaternions(&qx, &qy, &qf);
	//MultQuaternions(&qf, &qz, &qf);
	// ANOTHER TEST VARIATION
	//	MultQuaternions2(&qx,&qy,&qf);
	//	MultQuaternions2(&qf,&qz,&qf);

	// INSURE THE QUATERNION IS NORMALIZED
	// PROBABLY NOT NECESSARY IN MOST CASES

	normalise(qf);

	quat->q[0] = qf.q[0];
	quat->q[1] = qf.q[1];
	quat->q[2] = qf.q[2];
	quat->q[3] = qf.q[3];
}*/
// EulerToQuaternion2  /////////////////////////////////////////////////////////



/*void CheckDOFRestrictions(Bone *bone)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	vec3	euler;		// PLACE TO STORE EULER ANGLES
	///////////////////////////////////////////////////////////////////////////////

	// FIRST STEP IS TO CONVERT LINK QUATERNION BACK TO EULER ANGLES
	QuatToEuler(&bone->orientation, &euler);

	// CHECK THE DOF SETTINGS
	if (euler.v[0] > bone->max_rx)
		euler.v[0] = bone->max_rx;
	if (euler.v[0] < bone->min_rx)
		euler.v[0] = bone->min_rx;
	if (euler.v[1] > bone->max_ry)
		euler.v[1] = bone->max_ry;
	if (euler.v[1] < bone->min_ry)
		euler.v[1] = bone->min_ry;
	if (euler.v[2] > bone->max_rz)
		euler.v[2] = bone->max_rz;
	if (euler.v[2] < bone->min_rz)
		euler.v[2] = bone->min_rz;

	// BACK TO QUATERNION
	EulerToQuaternion2(&euler, &link->quat);
}*/

#define MAX_IK_TRIES 100 // TIMES THROUGH THE CCD LOOP
#define IK_POS_THRESH 0.1f // THRESHOLD FOR SUCCESS

///////////////////////////////////////////////////////////////////////////////
// Procedure:	ComputeCCDLink
// Purpose:		Compute an IK Solution to an end effector position in 3D
// Arguments:	End Target (x,y,z)
// Returns:		TRUE if a solution exists, FALSE if the position isn't in reach
///////////////////////////////////////////////////////////////////////////////	
bool Skeleton::CCDIK(vec3 position, int endEffectorIndex, int chainRootIndex)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	vec3		rootPos, curEnd, desiredEnd, targetVector, curVector, crossResult;
	GLfloat		cosAngle, turnAngle;
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
		rootPos.v[0] = bones[bone_index]->getGlobalTransformation().m[12];
		rootPos.v[1] = bones[bone_index]->getGlobalTransformation().m[13];
		rootPos.v[2] = bones[bone_index]->getGlobalTransformation().m[14];

		// POSITION OF THE END EFFECTOR
		curEnd.v[0] = bones[endEffectorIndex]->getGlobalTransformation().m[12];
		curEnd.v[1] = bones[endEffectorIndex]->getGlobalTransformation().m[13];
		curEnd.v[2] = bones[endEffectorIndex]->getGlobalTransformation().m[14];

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


				bones[bone_index]->rotateJoint(turnAngle, crossResult);

				// HANDLE THE DOF RESTRICTIONS IF I WANT THEM
				//if (m_DOF_Restrict)
				//	CheckDOFRestrictions(&bones[bone_index]);

				// RECALC ALL THE MATRICES WITHOUT DRAWING ANYTHING
				//drawScene(FALSE);		// CHANGE THIS TO TRUE IF YOU WANT TO SEE THE ITERATION
			}
			if (--bone_index < chainRootIndex) bone_index = endEffectorIndex - 1;	// START OF THE CHAIN, RESTART
		}
		// QUIT IF I AM CLOSE ENOUGH OR BEEN RUNNING LONG ENOUGH
	} while (tries++ < MAX_IK_TRIES &&
		get_squared_dist(curEnd, desiredEnd) > IK_POS_THRESH);
	if (tries == MAX_IK_TRIES)
		return FALSE;
	else
		return TRUE;
}