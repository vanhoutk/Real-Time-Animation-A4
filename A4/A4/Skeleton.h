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