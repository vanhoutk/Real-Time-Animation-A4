/*
*	Includes
*/
#include <assimp/cimport.h>		// C importer
#include <assimp/scene.h>		// Collects data
#include <assimp/postprocess.h> // Various extra operations
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <math.h>
#include <mmsystem.h>
#include <stdio.h>
#include <vector>				// STL dynamic memory
#include <windows.h>

#include "Antons_maths_funcs.h" // Anton's maths functions
#include "Camera.h"
#include "Mesh.h"
#include "PlaneRotation.h"
#include "Shader_Functions.h"
#include "Skeleton.h"
#include "time.h"

using namespace std;

/*
*	Globally defined variables and constants
*/
#define BUFFER_OFFSET(i) ((char *)NULL + (i))  // Macro for indexing vertex buffer

#define NUM_MESHES   12
#define NUM_SHADERS	 5
#define NUM_TEXTURES 1

bool firstMouse = true;
bool forwardAnimation = true;
bool keys[1024];
Camera camera(vec3(-1.5f, 2.0f, 30.0f));
enum Bezier { WAVE, CURVE };
enum Meshes { HAND_MESH, HAND_SHELL_MESH, JOINT_MESH, TIP_MESH, JOINT_SHELL_MESH, TIP_SHELL_MESH, LOWER_ARM_SHELL_MESH, UPPER_ARM_SHELL_MESH, TORSO_MESH, SHOULDERS_MESH, SHOULDERS_SHELL_MESH, SPHERE_MESH };
enum Modes { ROTATE_HAND, CLOSE_FIST, OPEN_FIST, CLOSE_AND_OPEN_FIST, ANALYTICAL_IK_2D, CCD_IK_MANUAL, CCD_IK_SPLINE, CCD_IK_MANUAL_FINGER, CCD_IK_SPLINE_FINGER};
enum Shaders { SKYBOX, BASIC_COLOUR_SHADER, BASIC_TEXTURE_SHADER, LIGHT_SHADER, LIGHT_TEXTURE_SHADER };
enum Textures { METAL_TEXTURE };
GLfloat cameraSpeed = 0.005f;
GLfloat currentTime = 0.0f;
GLfloat timeChange = 0.002f;
GLfloat yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
GLuint animationMode = -1;
GLuint bezierCurve = CURVE;
GLuint lastX = 400, lastY = 300;
GLuint shaderProgramID[NUM_SHADERS];
int screenWidth = 1000;
int screenHeight = 800;
Mesh handMesh, handShellMesh, fingerJointMesh, fingerTipMesh, jointShellMesh, tipShellMesh, lowerArmShellMesh, upperArmShellMesh, torsoMesh, shouldersMesh, shouldersShellMesh, sphereMesh;
Skeleton handSkeleton, torsoSkeleton;

vec3 spherePosition;// = vec3(-9.0f, 10.0f, 0.0f);
// | Spline points
vec3 p1 = vec3(-10.0f, 8.0f, 0.0f);
vec3 p2 = vec3(-11.0f, 11.0f, 1.0f);
vec3 p3 = vec3(-9.0f, 14.0f, 2.0f);
vec3 p4 = vec3(-2.0f, 10.0f, 3.0f);

vec3 wave[4] = { vec3(-9.0f, 8.0f, 4.0f), vec3(-7.0f, 11.0f, 4.0f), vec3(-5.0f, 13.0f, 4.0f), vec3(-4.0f, 10.0f, 4.0f) };
vec3 curve1[4] = { vec3(-6.0f, 6.0f, 4.0f), vec3(-9.0f, 6.0f, 4.0f), vec3(-9.0f, 10.0f, 4.0f), vec3(-6.0f, 10.0f, 4.0f) };
vec3 curve2[4] = { vec3(-6.0f, 10.0f, 4.0f), vec3(-3.0f, 10.0f, 4.0f), vec3(-3.0f, 6.0f, 4.0f), vec3(-6.0f, 6.0f, 4.0f) };

// | Resource Locations
const char * meshFiles[NUM_MESHES] = { "../Meshes/right_hand.obj", "../Meshes/right_hand_shell.obj", "../Meshes/finger_joint.dae", "../Meshes/finger_tip.dae", "../Meshes/finger_joint_shell.obj", "../Meshes/finger_tip_shell.dae", "../Meshes/lower_arm_shell.dae", "../Meshes/right_upper_arm_shell.dae", "../Meshes/torso_disk3.dae", "../Meshes/shoulders.obj", "../Meshes/shoulder_shell.dae", "../Meshes/particle.dae" };
const char * skyboxTextureFiles[6] = { "../Textures/TCWposx.png", "../Textures/TCWnegx.png", "../Textures/TCWposy.png", "../Textures/TCWnegy.png", "../Textures/TCWposz.png", "../Textures/TCWnegz.png" };
const char * textureFiles[NUM_TEXTURES] = { "../Textures/metal.jpg" };

const char * vertexShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxVertexShader.txt", "../Shaders/ParticleVertexShader.txt", "../Shaders/BasicTextureVertexShader.txt", "../Shaders/LightVertexShader.txt", "../Shaders/LightTextureVertexShader.txt" };
const char * fragmentShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxFragmentShader.txt", "../Shaders/ParticleFragmentShader.txt", "../Shaders/BasicTextureFragmentShader.txt", "../Shaders/LightFragmentShader.txt", "../Shaders/LightTextureFragmentShader.txt" };

void display()
{
	// Tell GL to only draw onto a pixel if the shape is closer to the viewer
	glEnable(GL_DEPTH_TEST);	// Enable depth-testing
	glDepthFunc(GL_LESS);		// Depth-testing interprets a smaller value as "closer"
	glClearColor(5.0f / 255.0f, 1.0f / 255.0f, 15.0f / 255.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set the view matrix to first or third person views
	mat4 view = camera.GetViewMatrix();

	// Projection Matrix
	mat4 projection = perspective(camera.Zoom, (float)screenWidth / (float)screenHeight, 0.1f, 100.0f);
	
	// Model Matrix
	mat4 model = identity_mat4();

	vec4 view_position = vec4(camera.Position.v[0], camera.Position.v[1], camera.Position.v[2], 0.0f);

	//handMesh.drawMesh(view, projection, model);
	torsoSkeleton.drawSkeleton(view, projection, view_position);

	mat4 sphere_model = scale(identity_mat4(), vec3(0.4f, 0.4f, 0.4f));
	sphere_model = translate(sphere_model, spherePosition);
	sphereMesh.drawMesh(view, projection, sphere_model, vec4(0.0f, 1.0f, 0.0f, 1.0f));

	glutSwapBuffers();
}

void processInput()
{
	if (keys[GLUT_KEY_UP])
		camera.ProcessKeyboard(FORWARD, cameraSpeed);
	if (keys[GLUT_KEY_DOWN])
		camera.ProcessKeyboard(BACKWARD, cameraSpeed);
	if (keys[GLUT_KEY_LEFT])
		camera.ProcessKeyboard(LEFT, cameraSpeed);
	if (keys[GLUT_KEY_RIGHT])
		camera.ProcessKeyboard(RIGHT, cameraSpeed);

	// Move the sphere position around manually
	if (animationMode == CCD_IK_MANUAL || animationMode == CCD_IK_MANUAL_FINGER)
	{
		if (keys['p'])
			spherePosition += vec3(0.01f, 0.0f, 0.0f);
		if (keys['o'])
			spherePosition += vec3(0.0f, 0.01f, 0.0f);
		if (keys['l'])
			spherePosition += vec3(0.0f, -0.01f, 0.0f);
		if (keys['i'])
			spherePosition += vec3(-0.01f, 0.0f, 0.0f);
		if (keys['j'])
			spherePosition += vec3(0.0f, 0.0f, 0.01f);
		if (keys['k'])
			spherePosition += vec3(0.0f, 0.0f, -0.01f);
	}

	// Change animation mode
	if (keys['1'])
		animationMode = ROTATE_HAND;
	if (keys['2'])
		animationMode = CLOSE_FIST;
	if (keys['3'])
		animationMode = OPEN_FIST;
	if (keys['4'])
		animationMode = CLOSE_AND_OPEN_FIST;
	//if (keys['5'])
		//animationMode = ANALYTICAL_IK_2D;
	if (keys['6'])
		animationMode = CCD_IK_MANUAL;
	if (keys['7'])
		animationMode = CCD_IK_SPLINE;
	if (keys['8'])
		animationMode = CCD_IK_MANUAL_FINGER;
	if (keys['9'])
		animationMode = CCD_IK_SPLINE_FINGER;
	if (keys['0'])
		animationMode = -1;

	if (keys['n'])
		bezierCurve = WAVE;
	if (keys['m'])
		bezierCurve = CURVE;

	// Close the window if 'Esc' is pressed
	if (keys[(char)27])
		exit(0);
}


void updatePosition()
{
	if (currentTime <= 1.0f)
		currentTime += timeChange;
	else
	{
		currentTime = 0.0f;
		forwardAnimation = !forwardAnimation;
	}

	//if(forwardAnimation)
	//	spherePosition = splinePositionBezier(p1, p2, p3, p4, currentTime);
	//else
	//	spherePosition = splinePositionBezier(p4, p3, p2, p1, currentTime);
	if (bezierCurve == WAVE)
	{
		if (forwardAnimation)
			spherePosition = splinePositionBezier(wave[0], wave[1], wave[2], wave[3], currentTime);
		else
			spherePosition = splinePositionBezier(wave[3], wave[2], wave[1], wave[0], currentTime);
	}
	else
	{
		if (forwardAnimation)
			spherePosition = splinePositionBezier(curve1[0], curve1[1], curve1[2], curve1[3], currentTime);
		else
			spherePosition = splinePositionBezier(curve2[0], curve2[1], curve2[2], curve2[3], currentTime);
	}
	


}

void updateScene()
{
	switch (animationMode)
	{
	case ROTATE_HAND:
		torsoSkeleton.rotateWrist360();
		break;
	case CLOSE_FIST:
		torsoSkeleton.closeFist();
		break;
	case OPEN_FIST:
		torsoSkeleton.openFist();
		break;
	case CLOSE_AND_OPEN_FIST:
		torsoSkeleton.closeAndOpenFist();
		break;
	case ANALYTICAL_IK_2D:
		torsoSkeleton.moveTo(spherePosition);
		break;
	case CCD_IK_MANUAL:
		torsoSkeleton.moveToCCD(spherePosition, true);
		break;
	case CCD_IK_SPLINE:
		updatePosition();
		torsoSkeleton.moveToCCD(spherePosition, true);
		break;
	case CCD_IK_MANUAL_FINGER:
		torsoSkeleton.moveToCCD(spherePosition, false);
		break;
	case CCD_IK_SPLINE_FINGER:
		updatePosition();
		torsoSkeleton.moveToCCD(spherePosition, false);
		break;
	}
	processInput();
	// Draw the next frame
	glutPostRedisplay();
}

void initialiseMeshes()
{
	handMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	handMesh.generateObjectBufferMesh(meshFiles[HAND_MESH]);
	handMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	fingerJointMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	fingerJointMesh.generateObjectBufferMesh(meshFiles[JOINT_MESH]);
	fingerJointMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	fingerTipMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	fingerTipMesh.generateObjectBufferMesh(meshFiles[TIP_MESH]);
	fingerTipMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	torsoMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	torsoMesh.generateObjectBufferMesh(meshFiles[TORSO_MESH]);
	torsoMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	shouldersMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	shouldersMesh.generateObjectBufferMesh(meshFiles[SHOULDERS_MESH]);
	shouldersMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	handShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	handShellMesh.generateObjectBufferMesh(meshFiles[HAND_SHELL_MESH]);

	jointShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	jointShellMesh.generateObjectBufferMesh(meshFiles[JOINT_SHELL_MESH]);

	tipShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	tipShellMesh.generateObjectBufferMesh(meshFiles[TIP_SHELL_MESH]);

	lowerArmShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	lowerArmShellMesh.generateObjectBufferMesh(meshFiles[LOWER_ARM_SHELL_MESH]);

	upperArmShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	upperArmShellMesh.generateObjectBufferMesh(meshFiles[UPPER_ARM_SHELL_MESH]);

	shouldersShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	shouldersShellMesh.generateObjectBufferMesh(meshFiles[SHOULDERS_SHELL_MESH]);

	sphereMesh = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	sphereMesh.generateObjectBufferMesh(meshFiles[SPHERE_MESH]);
}

void init()
{
	spherePosition = p1;

	// Compile the shaders
	for (int i = 0; i < NUM_SHADERS; i++)
	{
	shaderProgramID[i] = CompileShaders(vertexShaderNames[i], fragmentShaderNames[i]);
	}

	// Create all of the meshes
	initialiseMeshes();

	//handSkeleton.createRightHand(handMesh, handShellMesh, fingerJointMesh, jointShellMesh, fingerTipMesh, tipShellMesh);
	torsoSkeleton.createTorso(torsoMesh, shouldersMesh, shouldersShellMesh, upperArmShellMesh, lowerArmShellMesh, handMesh, handShellMesh, fingerJointMesh, jointShellMesh, fingerTipMesh, tipShellMesh);
}

/*
*	User Input Functions
*/
#pragma region USER_INPUT_FUNCTIONS
void pressNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = true;
}

void releaseNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = false;
}

void pressSpecialKeys(int key, int x, int y)
{
	keys[key] = true;
}

void releaseSpecialKeys(int key, int x, int y)
{
	keys[key] = false;
}

void mouseClick(int button, int state, int x, int y)
{}

void processMouse(int x, int y)
{
	if (firstMouse)
	{
		lastX = x;
		lastY = y;
		firstMouse = false;
	}

	int xoffset = x - lastX;
	int yoffset = lastY - y;

	lastX = x;
	lastY = y;

	camera.ProcessMouseMovement((GLfloat)xoffset, (GLfloat)yoffset);
}

void mouseWheel(int button, int dir, int x, int y)
{}
#pragma endregion

/*
*	Main
*/
int main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - screenWidth) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - screenHeight) / 4);
	glutCreateWindow("Hand Hierarchy");

	// Glut display and update functions
	glutDisplayFunc(display);
	glutIdleFunc(updateScene);

	// User input functions
	glutKeyboardFunc(pressNormalKeys);
	glutKeyboardUpFunc(releaseNormalKeys);
	glutSpecialFunc(pressSpecialKeys);
	glutSpecialUpFunc(releaseSpecialKeys);
	glutMouseFunc(mouseClick);
	glutPassiveMotionFunc(processMouse);
	glutMouseWheelFunc(mouseWheel);

	glewExperimental = GL_TRUE; //for non-lab machines, this line gives better modern GL support

	// A call to glewInit() must be done after glut is initialized!
	GLenum res = glewInit();
	// Check for any errors
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return 1;
	}

	// Set up meshes and shaders
	init();
	// Begin infinite event loop
	glutMainLoop();
	return 0;
}