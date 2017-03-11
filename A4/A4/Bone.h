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
#include "Mesh.h"
#include "PlaneRotation.h"

using namespace std;

class Bone {
public:
	Bone();
	Bone(string name, Bone* parent, mat4 initial_offset, Mesh joint, Mesh shell, bool hasShell, vec4 jointColour, vec4 shellColour);
	Bone(string name, bool isRoot, mat4 initial_offset, Mesh joint, Mesh shell, bool hasShell, vec4 jointColour, vec4 shellColour);

	mat4 getGlobalTransformation();
	vec4 getPosition();

	void addChild(Bone* child);
	void addChild(string name, mat4 initial_offset, Mesh joint, Mesh shell, bool hasShell);
	void drawBone(mat4 view, mat4 projection, vec4 viewPosition);
	void bendJoint(GLfloat rotation);
	void rollJoint(GLfloat rotation);
	void rotateJoint(GLfloat rotation, vec3 axis);
	void pivotJoint(GLfloat rotation);

	versor orientation;

private:
	bool hasShell;
	bool isRoot;
	Mesh joint;
	Mesh shell;
	vec4 jointColour;
	vec4 shellColour;

	string name;
	Bone* parent;
	vector<Bone*> children;
	
	
	vec4 upVector, rightVector, forwardVector;
	mat4 rotationMatrix;

	mat4 local_transformation;


};

Bone::Bone()
{

}

Bone::Bone(string name, Bone* parent, mat4 initial_offset, Mesh joint, Mesh shell = Mesh(), bool hasShell = false, vec4 jointColour = vec4(0.0f, 0.0f, 0.0f, 0.0f), vec4 shellColour = vec4(0.0f, 0.0f, 0.0f, 0.0f))
{
	this->name = name;
	this->isRoot = false;
	this->parent = parent;
	this->local_transformation = initial_offset;
	this->joint = joint;
	this->shell = shell;
	this->hasShell = hasShell;
	this->jointColour = jointColour;
	this->shellColour = shellColour;

	this->orientation = quat_from_axis_deg(0.0f, this->rightVector.v[0], this->rightVector.v[1], this->rightVector.v[2]);
	this->rotationMatrix = quat_to_mat4(this->orientation);
	this->forwardVector = this->rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	this->rightVector = this->rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	this->upVector = this->rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

Bone::Bone(string name, bool isRoot, mat4 initial_offset, Mesh joint, Mesh shell = Mesh(), bool hasShell = false, vec4 jointColour = vec4(0.0f, 0.0f, 0.0f, 0.0f), vec4 shellColour = vec4(0.0f, 0.0f, 0.0f, 0.0f))
{
	this->name = name;
	this->isRoot = isRoot;
	this->parent = NULL;
	this->local_transformation = initial_offset;
	this->joint = joint;
	this->shell = shell;
	this->hasShell = hasShell;
	this->jointColour = jointColour;
	this->shellColour = shellColour;

	this->orientation = quat_from_axis_deg(0.0f, this->rightVector.v[0], this->rightVector.v[1], this->rightVector.v[2]);
	this->rotationMatrix = quat_to_mat4(this->orientation);
	this->forwardVector = this->rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	this->rightVector = this->rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	this->upVector = this->rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

mat4 Bone::getGlobalTransformation()
{
	if (isRoot)
		return this->local_transformation * this->rotationMatrix;
	else
		return this->parent->getGlobalTransformation() * this->local_transformation * this->rotationMatrix;
}

vec4 Bone::getPosition()
{
	return getGlobalTransformation() * vec4(0.0, 0.0, 0.0, 1.0);
}


void Bone::addChild(Bone* child)
{
	this->children.push_back(child);
}

void Bone::addChild(string name, mat4 initial_offset, Mesh joint, Mesh shell = Mesh(), bool hasShell = false)
{
	Bone child = Bone(name, this, initial_offset, joint, shell, hasShell);
	this->children.push_back(&child);
}

void Bone::drawBone(mat4 view, mat4 projection, vec4 viewPosition = vec4(0.0f, 0.0f, 0.0f, 0.0f))
{
	mat4 model = this->getGlobalTransformation();
	this->joint.drawMesh(view, projection, model, this->jointColour, viewPosition);
	if (hasShell)
	{
		this->shell.drawMesh(view, projection, model, this->shellColour, viewPosition);
	}

	for (GLuint i = 0; i < this->children.size(); i++)
		this->children[i]->drawBone(view, projection, viewPosition);
}

void Bone::bendJoint(GLfloat rotation)
{
	versor quat = quat_from_axis_rad(rotation, this->rightVector.v[0], this->rightVector.v[1], this->rightVector.v[2]);
	multiplyQuat(this->orientation, quat, this->orientation);
	this->rotationMatrix = quat_to_mat4(this->orientation);
	this->forwardVector = this->rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	//this->rightVector = this->rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	this->upVector = this->rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

void Bone::rollJoint(GLfloat rotation)
{
	versor quat = quat_from_axis_rad(rotation, this->forwardVector.v[0], this->forwardVector.v[1], this->forwardVector.v[2]);
	multiplyQuat(this->orientation, quat, this->orientation);
	this->rotationMatrix = quat_to_mat4(this->orientation);
	//this->forwardVector = this->rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	this->rightVector = this->rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	this->upVector = this->rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

void Bone::pivotJoint(GLfloat rotation)
{
	versor quat = quat_from_axis_rad(rotation, this->upVector.v[0], this->upVector.v[1], this->upVector.v[2]);
	multiplyQuat(this->orientation, quat, this->orientation);
	this->rotationMatrix = quat_to_mat4(this->orientation);
	this->forwardVector = this->rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	this->rightVector = this->rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	//this->upVector = this->rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

void Bone::rotateJoint(GLfloat rotation, vec3 axis)
{
	vec3 localAxis = this->getGlobalTransformation() * vec4(axis.v[0], axis.v[1], axis.v[2], 0.0f);
	localAxis = normalise(localAxis);
	versor quat = quat_from_axis_rad(rotation, localAxis.v[0], localAxis.v[1], localAxis.v[2]);
	multiplyQuat(this->orientation, quat, this->orientation);
	this->rotationMatrix = quat_to_mat4(this->orientation);
	this->forwardVector = this->rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	this->rightVector = this->rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	this->upVector = this->rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}