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
	//void createLeftHand(Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell, int startIndex);
	void createRightHand(Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell, int startIndex);
	void createTorso(Mesh torsoMesh, Mesh shoulderMesh, Mesh shoulderShell, Mesh upperArmShell, Mesh lowerArmShell, Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell);
	void drawSkeleton(mat4 view, mat4 projection, vec4 viewPosition);
	void rotateWrist360();
	void closeFist();
	void openFist();
	void closeAndOpenFist();
	void moveTo(vec3 position);
	void moveToCCD(vec3 position);

private:
	bool close = true;
	bool once = false;

	GLfloat count = 0.0f;
	GLfloat speed = 0.15f;
	GLfloat theta1 = 0.0f;
	GLfloat theta2 = 0.0f;

	vec4 joint_colour = vec4(1.0f, 1.0f, 0.0f, 1.0f);
	vec4 shell_colour = vec4(1.0f, 1.0f, 1.0f, 1.0f);
};

Skeleton::Skeleton()
{

}

/*void Skeleton::createLeftHand(Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell, int startIndex = 0)
{
	// Hand
	if (startIndex == 0)
	{
		bones[startIndex + 0] = new Bone("right_hand", true, scale(identity_mat4(), vec3(0.2f, 0.2f, 0.2f)), handMesh, handShell, true, joint_colour, shell_colour);
		rootBone = bones[startIndex + 0];
	}
	else
	{
		mat4 hand_local = scale(identity_mat4(), vec3(0.3f, 0.3f, 0.3f));
		hand_local = rotate_y_deg(hand_local, 180.0f);
		hand_local = translate(hand_local, vec3(-7.0f, 0.0f, 0.0f));
		bones[startIndex + 0] = new Bone("right_hand", bones[startIndex - 1], hand_local, handMesh, handShell, true, joint_colour, shell_colour);
		bones[startIndex - 1]->addChild(bones[startIndex + 0]);
		bones[startIndex + 0]->rollJoint(radians(90.0f));
	}

	// Thumb
	mat4 thumb1_local = scale(identity_mat4(), vec3(0.75f, 1.0f, 1.0f) * 0.8f);
	thumb1_local = translate(thumb1_local, vec3(3.0f, 0.0f, 2.5f));
	bones[startIndex + 1] = new Bone("right_thumb_1", bones[startIndex + 0], thumb1_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 1]);
	bones[startIndex + 1]->pivotJoint(radians(30.0f));
	bones[startIndex + 1]->rollJoint(radians(45.0f));

	mat4 thumb2_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 2] = new Bone("right_thumb_2", bones[startIndex + 1], thumb2_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 1]->addChild(bones[startIndex + 2]);
	bones[startIndex + 2]->bendJoint(radians(2.0f));

	mat4 thumb3_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 3] = new Bone("right_thumb_3", bones[startIndex + 2], thumb3_local, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 2]->addChild(bones[startIndex + 3]);
	bones[startIndex + 3]->bendJoint(radians(5.0f));

	// Finger 1
	mat4 finger11_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger11_local = translate(finger11_local, vec3(-4.0f, 0.0f, 3.1f));
	bones[startIndex + 4] = new Bone("right_finger_11", bones[startIndex + 0], finger11_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 4]);

	mat4 finger_joint_offset = scale(identity_mat4(), vec3(0.85f, 0.85f, 0.85f));
	finger_joint_offset = translate(finger_joint_offset, vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 5] = new Bone("right_finger_12", bones[startIndex + 4], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 4]->addChild(bones[startIndex + 5]);

	mat4 finger_tip_offset = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 6] = new Bone("right_finger_13", bones[startIndex + 5], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 5]->addChild(bones[startIndex + 6]);

	// Finger 2
	mat4 finger21_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f) * 0.8f);
	finger21_local = translate(finger21_local, vec3(-4.0f, 0.0f, 0.9f));
	bones[startIndex + 7] = new Bone("right_finger_21", bones[startIndex + 0], finger21_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 7]);

	bones[startIndex + 8] = new Bone("right_finger_22", bones[startIndex + 7], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 7]->addChild(bones[startIndex + 8]);

	bones[startIndex + 9] = new Bone("right_finger_23", bones[startIndex + 8], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 8]->addChild(bones[startIndex + 9]);

	// Finger 3
	mat4 finger31_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger31_local = translate(finger31_local, vec3(-4.0f, 0.0f, -1.3f));
	bones[startIndex + 10] = new Bone("right_finger_31", bones[startIndex + 0], finger31_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 10]);

	bones[startIndex + 11] = new Bone("right_finger_32", bones[startIndex + 10], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 10]->addChild(bones[startIndex + 11]);

	bones[startIndex + 12] = new Bone("right_finger_33", bones[startIndex + 11], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 11]->addChild(bones[startIndex + 12]);

	// Finger 4
	mat4 finger41_local = scale(identity_mat4(), vec3(0.7f, 0.7f, 0.7f) * 0.8f);
	finger41_local = translate(finger41_local, vec3(-4.0f, 0.0f, -3.1f));
	bones[startIndex + 13] = new Bone("right_finger_41", bones[startIndex + 0], finger41_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 13]);

	bones[startIndex + 14] = new Bone("right_finger_42", bones[startIndex + 13], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 13]->addChild(bones[startIndex + 14]);

	bones[startIndex + 15] = new Bone("right_finger_43", bones[startIndex + 14], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 14]->addChild(bones[startIndex + 15]);
}*/

void Skeleton::createRightHand(Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell, int startIndex = 0)
{
	// Hand
	if (startIndex == 0)
	{
		bones[startIndex + 0] = new Bone("right_hand", true, scale(identity_mat4(), vec3(0.2f, 0.2f, 0.2f)), handMesh, handShell, true, joint_colour, shell_colour);
		rootBone = bones[startIndex + 0];
	}
	else
	{
		mat4 hand_local = scale(identity_mat4(), vec3(0.3f, 0.3f, 0.3f));
		hand_local = translate(hand_local, vec3(-6.0f, 0.0f, 0.0f));
		bones[startIndex + 0] = new Bone("right_hand", bones[startIndex - 1], hand_local, handMesh, handShell, true, joint_colour, shell_colour);
		bones[startIndex - 1]->addChild(bones[startIndex + 0]);
		bones[startIndex + 0]->rollJoint(radians(90.0f));
	}

	// Thumb
	mat4 thumb1_local = scale(identity_mat4(), vec3(0.75f, 1.0f, 1.0f) * 0.8f);
	thumb1_local = translate(thumb1_local, vec3(-1.0f, 0.0f, 2.5f));
	bones[startIndex + 1] = new Bone("right_thumb_1", bones[startIndex + 0], thumb1_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 1]);
	bones[startIndex + 1]->pivotJoint(radians(30.0f));
	bones[startIndex + 1]->rollJoint(radians(45.0f));

	mat4 thumb2_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 2] = new Bone("right_thumb_2", bones[startIndex + 1], thumb2_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 1]->addChild(bones[startIndex + 2]);
	bones[startIndex + 2]->bendJoint(radians(2.0f));

	mat4 thumb3_local = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 3] = new Bone("right_thumb_3", bones[startIndex + 2], thumb3_local, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 2]->addChild(bones[startIndex + 3]);
	bones[startIndex + 3]->bendJoint(radians(5.0f));

	// Finger 1
	mat4 finger11_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger11_local = translate(finger11_local, vec3(-8.0f, 0.0f, 3.1f));
	bones[startIndex + 4] = new Bone("right_finger_11", bones[startIndex + 0], finger11_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 4]);

	mat4 finger_joint_offset = scale(identity_mat4(), vec3(0.85f, 0.85f, 0.85f));
	finger_joint_offset = translate(finger_joint_offset, vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 5] = new Bone("right_finger_12", bones[startIndex + 4], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 4]->addChild(bones[startIndex + 5]);

	mat4 finger_tip_offset = translate(identity_mat4(), vec3(-5.0f, 0.0f, 0.0f));
	bones[startIndex + 6] = new Bone("right_finger_13", bones[startIndex + 5], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 5]->addChild(bones[startIndex + 6]);

	// Finger 2
	mat4 finger21_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f) * 0.8f);
	finger21_local = translate(finger21_local, vec3(-8.0f, 0.0f, 0.9f));
	bones[startIndex + 7] = new Bone("right_finger_21", bones[startIndex + 0], finger21_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 7]);

	bones[startIndex + 8] = new Bone("right_finger_22", bones[startIndex + 7], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 7]->addChild(bones[startIndex + 8]);

	bones[startIndex + 9] = new Bone("right_finger_23", bones[startIndex + 8], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 8]->addChild(bones[startIndex + 9]);

	// Finger 3
	mat4 finger31_local = scale(identity_mat4(), vec3(0.9f, 0.9f, 0.9f) * 0.8f);
	finger31_local = translate(finger31_local, vec3(-8.0f, 0.0f, -1.3f));
	bones[startIndex + 10] = new Bone("right_finger_31", bones[startIndex + 0], finger31_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 10]);

	bones[startIndex + 11] = new Bone("right_finger_32", bones[startIndex + 10], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 10]->addChild(bones[startIndex + 11]);

	bones[startIndex + 12] = new Bone("right_finger_33", bones[startIndex + 11], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 11]->addChild(bones[startIndex + 12]);

	// Finger 4
	mat4 finger41_local = scale(identity_mat4(), vec3(0.7f, 0.7f, 0.7f) * 0.8f);
	finger41_local = translate(finger41_local, vec3(-8.0f, 0.0f, -3.1f));
	bones[startIndex + 13] = new Bone("right_finger_41", bones[startIndex + 0], finger41_local, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 0]->addChild(bones[startIndex + 13]);

	bones[startIndex + 14] = new Bone("right_finger_42", bones[startIndex + 13], finger_joint_offset, jointMesh, jointShell, true, joint_colour, shell_colour);
	bones[startIndex + 13]->addChild(bones[startIndex + 14]);

	bones[startIndex + 15] = new Bone("right_finger_43", bones[startIndex + 14], finger_tip_offset, tipMesh, tipShell, true, joint_colour, shell_colour);
	bones[startIndex + 14]->addChild(bones[startIndex + 15]);
}

void Skeleton::createTorso(Mesh torsoMesh, Mesh shoulderMesh, Mesh shoulderShell, Mesh upperArmShell, Mesh lowerArmShell, Mesh handMesh, Mesh handShell, Mesh jointMesh, Mesh jointShell, Mesh tipMesh, Mesh tipShell)
{

	// Torso
	mat4 torso_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f));
	//torso_local = translate(torso_local, vec3(-5.0f, 0.0f, 0.0f));
	bones[0] = new Bone("torso", true, torso_local, torsoMesh, Mesh(), false, joint_colour);
	rootBone = bones[0];

	// Shoulders
	mat4 shoulder_local = scale(identity_mat4(), vec3(1.0f, 1.0f, 1.0f));
	shoulder_local = translate(shoulder_local, vec3(0.0f, 7.5f, 0.0f));
	bones[1] = new Bone("shoulders", bones[0], shoulder_local, shoulderMesh, shoulderShell, true, joint_colour, shell_colour);
	bones[0]->addChild(bones[1]);

	// Upper Arm
	mat4 upper_arm_local = scale(identity_mat4(), vec3(1.2f, 1.2f, 1.2f) * 0.8f);
	upper_arm_local = translate(upper_arm_local, vec3(-2.5f, 0.0f, 0.0f));
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

	createRightHand(handMesh, handShell, jointMesh, jointShell, tipMesh, tipShell, 4);
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
	// Undo previous rotation
	bones[3]->pivotJoint(1.0f * radians(theta2));
	bones[2]->pivotJoint(1.0f * radians(theta1));

	vec3 start_position = vec3(-2.5f, 7.5f, 0.0f);
	analyticalIK(position - start_position, 5.0f, 5.0f, theta1, theta2);

	// Rotate bones
	bones[2]->pivotJoint(-1.0f * radians(theta1));
	bones[3]->pivotJoint(-1.0f * radians(theta2));
}

void Skeleton::moveToCCD(vec3 position)
{
	CCDIK(position, handIndex, handIndex - 2, bones);
}