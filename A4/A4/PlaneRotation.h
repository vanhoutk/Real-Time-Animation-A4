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
#include "PlaneRotation.h"

GLfloat radians(GLfloat degrees)
{
	return (degrees * ((2.0f * (float)PI) / 360.0f));
}

mat4 getRotationMatrix(GLfloat yawR, GLfloat pitchR, GLfloat rollR, vec4 &fV, vec4 &rightV, vec4 &upV)
{
	// Tait-Bryan Angles ZYX
	mat4 rotationMatrix = identity_mat4();
	rotationMatrix.m[0] = cos(pitchR) * cos(yawR);
	rotationMatrix.m[1] = sin(pitchR) * cos(yawR);
	rotationMatrix.m[2] = -1 * sin(yawR);

	rotationMatrix.m[4] = (cos(pitchR) * sin(yawR) * sin(rollR)) - (sin(pitchR) * cos(rollR));
	rotationMatrix.m[5] = (sin(pitchR) * sin(yawR) * sin(rollR)) + (cos(pitchR) * cos(rollR));
	rotationMatrix.m[6] = cos(yawR) * sin(rollR);

	rotationMatrix.m[8] = (cos(pitchR) * sin(yawR) * cos(rollR)) + (sin(pitchR) * sin(rollR));
	rotationMatrix.m[9] = (sin(pitchR) * sin(yawR) * cos(rollR)) - (cos(pitchR) * sin(rollR));
	rotationMatrix.m[10] = cos(yawR) * cos(rollR);

	fV = rotationMatrix * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	rightV = rotationMatrix * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	upV = rotationMatrix * vec4(0.0f, 1.0f, 0.0f, 0.0f);

	return rotationMatrix;
}

/*void applyEulerYaw(GLfloat yaw, mat4 &rotationMat, vec4 &upV, vec4 &fV, vec4 &rightV)
{
	mat4 newRotation = getRotationMatrix(radians(yaw), 0.0f, 0.0f, upV, fV, rightV);
	rotationMat = newRotation * rotationMat;
}

void applyEulerRoll(GLfloat roll, mat4 &rotationMat, vec4 &upV, vec4 &fV, vec4 &rightV)
{
	mat4 newRotation = getRotationMatrix(0.0f, 0.0f, radians(roll), upV, fV, rightV);
	rotationMat = newRotation * rotationMat;
}

void applyEulerPitch(GLfloat pitch, mat4 &rotationMat, vec4 &upV, vec4 &fV, vec4 &rightV)
{
	mat4 newRotation = getRotationMatrix(0.0f, radians(pitch), 0.0f, upV, fV, rightV);
	rotationMat = newRotation * rotationMat;
}*/

void applyYaw(GLfloat yawR, mat4 &rotationMat, vec4 &upV, vec4 &fV, vec4 &rightV, versor &orientation)
{
	versor quat = quat_from_axis_rad(yawR, upV.v[0], upV.v[1], upV.v[2]);
	multiplyQuat(orientation, quat, orientation);
	rotationMat = quat_to_mat4(orientation);
	fV = rotationMat * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	rightV = rotationMat * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	upV = rotationMat * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

void applyRoll(GLfloat rollR, mat4 &rotationMat, vec4 &upV, vec4 &fV, vec4 &rightV, versor &orientation)
{
	versor quat = quat_from_axis_rad(rollR, fV.v[0], fV.v[1], fV.v[2]);
	multiplyQuat(orientation, quat, orientation);
	rotationMat = quat_to_mat4(orientation);
	fV = rotationMat * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	rightV = rotationMat * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	upV = rotationMat * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

void applyPitch(GLfloat pitchR, mat4 &rotationMat, vec4 &upV, vec4 &fV, vec4 &rightV, versor &orientation)
{
	versor quat = quat_from_axis_rad(pitchR, rightV.v[0], rightV.v[1], rightV.v[2]);
	multiplyQuat(orientation, quat, orientation);
	rotationMat = quat_to_mat4(orientation);
	fV = rotationMat * vec4(0.0f, 0.0f, 1.0f, 0.0f);
	rightV = rotationMat * vec4(1.0f, 0.0f, 0.0f, 0.0f);
	upV = rotationMat * vec4(0.0f, 1.0f, 0.0f, 0.0f);
}