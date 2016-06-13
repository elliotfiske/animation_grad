#include <iostream>
#include <vector>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "Camera.h"
#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Texture.h"
#include "Link.hpp"
#include "Scene.hpp"

using namespace std;
using namespace Eigen;

bool keyToggles[256] = {false}; // only for English keyboards!

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

shared_ptr<Program> progSimple;
shared_ptr<Program> progTex;
shared_ptr<Camera> camera;
shared_ptr<Shape> shape;
shared_ptr<Texture> texture;

shared_ptr<Scene> scene;

Vector2f mouse;



int muh_state = 0;
int MUH_BEFORE = 0;
int MUH_DRAGGING = 1;
int MUH_FLYING = 2;
int MUH_REPLAYING = 3;

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

static void char_callback(GLFWwindow *window, unsigned int key) {
	keyToggles[key] = !keyToggles[key];
   if (key == '1') {
      scene->make_links(1);
   }
   if (key == '2') {
      scene->make_links(2);
   }
   if (key == '3') {
      scene->make_links(3);
   }
   
   if (key == ' ') {
      muh_state = MUH_FLYING;
      scene->new_bomb(-camera->translations(0), -camera->translations(1), -camera->translations(2), camera->rotations(0), camera->rotations(1));
   }
   
   if (key == 'e') {
      scene->explode();
   }
}

float even_joint_angles = 0;
float odd_joint_angles = 0;

double thing = 32;

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if(state == GLFW_PRESS) {
		if(muh_state == MUH_BEFORE) {
         muh_state = MUH_DRAGGING;
      }
      else if (muh_state == MUH_DRAGGING) {
			// Move the links
			if(mouse(0) == 0.0f && mouse(1) == 0.0f) {
				// Initial call
				mouse << xmouse, ymouse;
			}
			float dx = xmouse - mouse(0);
			float dy = ymouse - mouse(1);
			float s = 0.01f;
			//
			// Use dx and dy to change the joint angles
			//
         even_joint_angles += dx * s;
         odd_joint_angles  += dy * s;
         
			cout << dx << " " << dy << endl;
			mouse << xmouse, ymouse;
         
         scene->burd.curr_E.block<3, 1>(0, 3) += Vector3d(dx/thing, -dy/ thing, 0);
         
         
		} else {
			camera->mouseMoved(xmouse, ymouse);
		}
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	// Get the current mouse position.
	double xmouse, ymouse;
	glfwGetCursorPos(window, &xmouse, &ymouse);
	// Get current window size.
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	if(action == GLFW_PRESS) {
		bool shift = mods & GLFW_MOD_SHIFT;
		bool ctrl  = mods & GLFW_MOD_CONTROL;
		bool alt   = mods & GLFW_MOD_ALT;
		camera->mouseClicked(xmouse, ymouse, shift, ctrl, alt);
	}
}

static void init()
{
	GLSL::checkVersion();
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	
	keyToggles[(unsigned)'c'] = true;
	
	progSimple = make_shared<Program>();
	progSimple->setShaderNames(RESOURCE_DIR + "simple_vert.glsl", RESOURCE_DIR + "simple_frag.glsl");
	progSimple->setVerbose(false); // Set this to true when debugging.
	progSimple->init();
	progSimple->addUniform("P");
	progSimple->addUniform("MV");
	
	progTex = make_shared<Program>();
	progTex->setVerbose(true); // Set this to true when debugging.
	progTex->setShaderNames(RESOURCE_DIR + "tex_vert.glsl", RESOURCE_DIR + "tex_frag.glsl");
	progTex->init();
	progTex->addUniform("P");
	progTex->addUniform("MV");
	progTex->addAttribute("vertPos");
	progTex->addAttribute("vertNor");
	progTex->addAttribute("vertTex");
	progTex->addUniform("texture0");
	
	texture = make_shared<Texture>();
	texture->setFilename(RESOURCE_DIR + "metal_texture_15_by_wojtar_stock.jpg");
	texture->init();
	
	shape = make_shared<Shape>();
	shape->loadMesh(RESOURCE_DIR + "cube.obj");
	shape->init();
   
   scene = make_shared<Scene>();
   scene->make_links(3);
	
	camera = make_shared<Camera>();
	
	// Initialize time.
	glfwSetTime(0.0);
	
	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
   // Step the link. NOTE: this is BAD you shouldn't update in the render loop.
   scene->step_all(0.016).cast<float>() - Vector3f(1.0f, 0.0f, 0.0f);
   
//   camera->translations[0] += 0.1f;
   
	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Use the window size for camera.
	glfwGetWindowSize(window, &width, &height);
	camera->setAspect((float)width/(float)height);
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyToggles[(unsigned)'c']) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyToggles[(unsigned)'l']) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
	auto P = make_shared<MatrixStack>();
	auto MV = make_shared<MatrixStack>();
	
	// Apply camera transforms
	P->pushMatrix();
	camera->applyProjectionMatrix(P);
	MV->pushMatrix();
	camera->applyViewMatrix(MV);
	
	// Draw grid
	progSimple->bind();
	glUniformMatrix4fv(progSimple->getUniform("P"), 1, GL_FALSE, P->topMatrix().data());
	glUniformMatrix4fv(progSimple->getUniform("MV"), 1, GL_FALSE, MV->topMatrix().data());
	glLineWidth(2.0f);
	float x0 = -2.0f;
	float x1 = 2.0f;
	float z0 = -2.0f;
	float z1 = 2.0f;
    float y = 0.0f;
	glColor3f(0.2f, 0.2f, 0.2f);
	glBegin(GL_LINE_LOOP);
	glVertex3f(x0, y, z0);
	glVertex3f(x1, y, z0);
	glVertex3f(x1, y, z1);
	glVertex3f(x0, y, z1);
	glEnd();
	int gridSize = 8;
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for(int i = 1; i < gridSize; ++i) {
		if(i == gridSize/2) {
			glColor3f(0.2f, 0.2f, 0.2f);
		} else {
			glColor3f(0.8f, 0.8f, 0.8f);
		}
		float x = x0 + i / (float)gridSize * (x1 - x0);
		glVertex3f(x, y, z0);
		glVertex3f(x, y, z1);
	}
	for(int i = 1; i < gridSize; ++i) {
		if(i == gridSize/2) {
			glColor3f(0.2f, 0.2f, 0.2f);
		} else {
			glColor3f(0.8f, 0.8f, 0.8f);
		}
		float z = z0 + i / (float)gridSize * (z1 - z0);
		glVertex3f(x0, y, z);
        glVertex3f(x1, y, z);
	}
	glEnd();
	progSimple->unbind();
   
   // Draw shape
   progTex->bind();
   texture->bind(progTex->getUniform("texture0"), 0);
   glUniformMatrix4fv(progTex->getUniform("P"), 1, GL_FALSE, P->topMatrix().data());
   
   scene->draw(MV.get(), progTex, shape);
   
   texture->unbind(0);
   progTex->unbind();
   
   //////////////////////////////////////////////////////
	// Cleanup
	//////////////////////////////////////////////////////
	
	// Pop stacks
	MV->popMatrix();
	P->popMatrix();
	
	GLSL::checkError(GET_FILE_LINE);
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		cout << "Please specify the resource directory." << endl;
		return 0;
	}
	RESOURCE_DIR = argv[1] + string("/");
	
	// Set error callback.
	glfwSetErrorCallback(error_callback);
	// Initialize the library.
	if(!glfwInit()) {
		return -1;
	}
	// Create a windowed mode window and its OpenGL context.
	window = glfwCreateWindow(640, 480, "ELLIOT FISKE", NULL, NULL);
	if(!window) {
		glfwTerminate();
		return -1;
	}
	// Make the window's context current.
	glfwMakeContextCurrent(window);
	// Initialize GLEW.
	glewExperimental = true;
	if(glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return -1;
	}
	glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	// Set vsync.
	glfwSwapInterval(1);
	// Set keyboard callback.
	glfwSetKeyCallback(window, key_callback);
	// Set char callback.
	glfwSetCharCallback(window, char_callback);
	// Set cursor position callback.
	glfwSetCursorPosCallback(window, cursor_position_callback);
	// Set mouse button callback.
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	// Initialize scene.
	init();
	// Loop until the user closes the window.
	while(!glfwWindowShouldClose(window)) {
		// Render scene.
		render();
		// Swap front and back buffers.
		glfwSwapBuffers(window);
		// Poll for and process events.
		glfwPollEvents();
	}
	// Quit program.
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
