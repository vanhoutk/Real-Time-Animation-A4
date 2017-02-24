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

#define NUM_MESHES   6
#define NUM_SHADERS	 5
#define NUM_TEXTURES 1

bool firstMouse = true;
bool keys[1024];
Camera camera(vec3(-1.5f, 2.0f, 10.0f));
//enum Meshes { BASE_MESH, THUMB0_MESH, THUMB1_MESH, THUMB2_MESH };
enum Meshes { HAND_MESH, HAND_SHELL_MESH, JOINT_MESH, TIP_MESH, JOINT_SHELL_MESH, TIP_SHELL_MESH };
enum Modes { ROTATE_HAND, CLOSE_FIST, OPEN_FIST, CLOSE_AND_OPEN_FIST};
enum Shaders { SKYBOX, BASIC_COLOUR_SHADER, BASIC_TEXTURE_SHADER, LIGHT_SHADER, LIGHT_TEXTURE_SHADER };
enum Textures { METAL_TEXTURE };
GLfloat cameraSpeed = 0.005f;
GLfloat yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
GLuint animationMode = -1;
GLuint boneIndex = 0;
GLuint lastX = 400, lastY = 300;
GLuint shaderProgramID[NUM_SHADERS];
int screenWidth = 1000;
int screenHeight = 800;
Mesh skyboxMesh;// , planeMesh;
//Mesh baseMesh, thumbMesh0, thumbMesh1, thumbMesh2;
Mesh handMesh, handShellMesh, fingerJointMesh, fingerTipMesh, jointShellMesh, tipShellMesh;
Skeleton handSkeleton;
vec4 upV = vec4(0.0f, 0.0f, 1.0f, 0.0f); //Up and Forward are flipped because of the initial rotation of the model
vec4 fV = vec4(0.0f, 1.0f, 0.0f, 0.0f);
vec4 rightV = vec4(1.0f, 0.0f, 0.0f, 0.0f);
vec3 origin = vec3(0.0f, 0.0f, 0.0f);
versor orientation;
mat4 rotationMat;
//mat4 eulerRotationMat;

// | Resource Locations
const char * meshFiles[NUM_MESHES] = { "../Meshes/hand.obj", "../Meshes/hand_shell.obj", "../Meshes/finger_joint.dae", "../Meshes/finger_tip.dae", "../Meshes/finger_joint_shell.obj", "../Meshes/finger_tip_shell.dae" };
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

	// Draw skybox first
	//skyboxMesh.drawSkybox(view, projection);

	vec4 view_position = vec4(camera.Position.v[0], camera.Position.v[1], camera.Position.v[2], 0.0f);

	handSkeleton.drawSkeleton(view, projection, view_position);

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

	/*if (keys['p'])
		handSkeleton.bones[boneIndex]->rollJoint(radians(1.0f));
	if (keys['o'])
		handSkeleton.bones[boneIndex]->bendJoint(radians(1.0f));
	if(keys['i'])
		handSkeleton.bones[boneIndex]->pivotJoint(radians(1.0f));*/

	if (keys['1'])
		animationMode = ROTATE_HAND;
	if (keys['2'])
		animationMode = CLOSE_FIST;
	if (keys['3'])
		animationMode = OPEN_FIST;
	if (keys['4'])
		animationMode = CLOSE_AND_OPEN_FIST;
	/*if (keys['5'])
		boneIndex = 5;
	if (keys['6'])
		boneIndex = 6;
	if (keys['7'])
		boneIndex = 7;
	if (keys['8'])
		boneIndex = 8;
	if (keys['9'])
		boneIndex = 9;*/
	if (keys['0'])
		animationMode = -1;


	if (keys[(char)27])
		exit(0);
}

void updateScene()
{
	switch (animationMode)
	{
	case ROTATE_HAND:
		handSkeleton.rotateWrist360();
		break;
	case CLOSE_FIST:
		handSkeleton.closeFist();
		break;
	case OPEN_FIST:
		handSkeleton.openFist();
		break;
	case CLOSE_AND_OPEN_FIST:
		handSkeleton.closeAndOpenFist();
		break;
	}
	processInput();
	// Draw the next frame
	glutPostRedisplay();
}

void init()
{
	// Compile the shaders
	for (int i = 0; i < NUM_SHADERS; i++)
	{
	shaderProgramID[i] = CompileShaders(vertexShaderNames[i], fragmentShaderNames[i]);
	}

	skyboxMesh = Mesh(&shaderProgramID[SKYBOX]);
	skyboxMesh.setupSkybox(skyboxTextureFiles);

	//planeMesh = Mesh(&shaderProgramID[BASIC_TEXTURE_SHADER]);
	//planeMesh.generateObjectBufferMesh(meshFiles[PLANE_MESH]);
	//planeMesh.loadTexture(textureFiles[PLANE_TEXTURE]);

	//baseMesh = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	//baseMesh.generateObjectBufferMesh(meshFiles[BASE_MESH]);

	handMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	handMesh.generateObjectBufferMesh(meshFiles[HAND_MESH]);
	handMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	fingerJointMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	fingerJointMesh.generateObjectBufferMesh(meshFiles[JOINT_MESH]);
	fingerJointMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	fingerTipMesh = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	fingerTipMesh.generateObjectBufferMesh(meshFiles[TIP_MESH]);
	fingerTipMesh.loadTexture(textureFiles[METAL_TEXTURE]);

	handShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	handShellMesh.generateObjectBufferMesh(meshFiles[HAND_SHELL_MESH]);

	jointShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	jointShellMesh.generateObjectBufferMesh(meshFiles[JOINT_SHELL_MESH]);

	tipShellMesh = Mesh(&shaderProgramID[LIGHT_SHADER]);
	tipShellMesh.generateObjectBufferMesh(meshFiles[TIP_SHELL_MESH]);

	handSkeleton.createHand(handMesh, handShellMesh, fingerJointMesh, jointShellMesh, fingerTipMesh, tipShellMesh);

	//thumbMesh0 = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	//thumbMesh0.generateObjectBufferMesh(meshFiles[THUMB0_MESH]);

	//thumbMesh1 = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	//thumbMesh1.generateObjectBufferMesh(meshFiles[THUMB1_MESH]);

	//thumbMesh2 = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	//thumbMesh2.generateObjectBufferMesh(meshFiles[THUMB2_MESH]);
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