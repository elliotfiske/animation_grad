#include <iostream>
#include <vector>
#include <random>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"

using namespace std;
using namespace Eigen;

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from
shared_ptr<Program> prog;

bool keyToggles[256] = {false}; // only for English keyboards!
Vector2f cameraRotations(0, 0);
Vector2f mousePrev(-1, -1);

// Control points
vector<Vector3f> cps;

enum SplineType
{
	BEZIER = 0,
	CATMULL_ROM,
	BASIS,
	SPLINE_TYPE_COUNT
};

SplineType type = BEZIER;

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
	
	if(action == GLFW_PRESS) {
		switch(key) {
			case GLFW_KEY_S:
				type = (SplineType)((type + 1) % SPLINE_TYPE_COUNT);
				break;
			case GLFW_KEY_C:
				cps.clear();
				break;
			case GLFW_KEY_R:
				cps.clear();
				default_random_engine generator;
				uniform_real_distribution<float> distribution(-0.8, 0.8);
				for(int i = 0; i < 10; ++i) {
					float x = distribution(generator);
					float y = distribution(generator);
					float z = distribution(generator);
					cps.push_back(Vector3f(x, y, z));
				}
				break;
		}
	}
}

static void char_callback(GLFWwindow *window, unsigned int key)
{
	keyToggles[key] = !keyToggles[key];
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	if(mousePrev(0) >= 0) {
		Vector2f mouseCurr;
		mouseCurr << xmouse, ymouse;
		cameraRotations += 0.5f * (mouseCurr - mousePrev);
		mousePrev = mouseCurr;
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
		if(mods & GLFW_MOD_SHIFT) {
			// Insert a new control point
			// Convert from window coord to world coord assuming that we're
			// using an orthgraphic projection from -1 to 1.
			float aspect = (float)width/height;
			Vector4f x;
			x(0) = 2.0f * ((xmouse / width) - 0.5f)* aspect;
			x(1) = 2.0f * (((height - ymouse) / height) - 0.5f);
			x(2) = 0.0f;
			x(3) = 1.0f;
			// Build the current modelview matrix.
			auto MV = make_shared<MatrixStack>();
			MV->rotate(cameraRotations(1), Vector3f(1, 0, 0));
			MV->rotate(cameraRotations(0), Vector3f(0, 1, 0));
			// Since the modelview matrix transforms from world to eye coords,
			// we want to invert to go from eye to world.
			x = MV->topMatrix().inverse() * x;
			cps.push_back(x.segment<3>(0));
		} else {
			mousePrev << xmouse, ymouse;
		}
	} else {
		mousePrev << -1, -1;
	}
}

static void init()
{
	GLSL::checkVersion();
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	
	// Initialize the GLSL program.
	prog = make_shared<Program>();
	prog->setVerbose(false); // Set this to true when debugging.
	prog->setShaderNames(RESOURCE_DIR + "lab02_vert.glsl", RESOURCE_DIR + "lab02_frag.glsl");
	prog->init();
	prog->addUniform("P");
	prog->addUniform("MV");
	
	// Initialize time.
	glfwSetTime(0.0);
	
	keyToggles[(unsigned)'l'] = true;

	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
	// Update time.
	double t = glfwGetTime();
	
	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	auto P = make_shared<MatrixStack>();
	auto MV = make_shared<MatrixStack>();
	P->pushMatrix();
	MV->pushMatrix();
	
	double aspect = (double)width/height;
	P->ortho(-aspect, aspect, -1.0, 1.0, -2.0, 2.0);
	MV->rotate(cameraRotations(1), Vector3f(1, 0, 0));
	MV->rotate(cameraRotations(0), Vector3f(0, 1, 0));
	
	// Bind the program
	prog->bind();
	glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, P->topMatrix().data());
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, MV->topMatrix().data());
	
	// Draw control points
	int ncps = (int)cps.size();
	glPointSize(5.0f);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for(int i = 0; i < ncps; ++i) {
		Vector3f cp = cps[i];
		glVertex3f(cp(0), cp(1), cp(2));
	}
	glEnd();
	glLineWidth(1.0f);
	if(keyToggles[(unsigned)'l']) {
		glColor3f(1.0f, 0.5f, 0.5f);
		glBegin(GL_LINE_STRIP);
		for(int i = 0; i < ncps; ++i) {
			Vector3f cp = cps[i];
			glVertex3f(cp(0), cp(1), cp(2));
		}
		glEnd();
	}

	// Unbind the program
	prog->unbind();

	// Pop matrix stacks.
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
	window = glfwCreateWindow(640, 480, "YOUR NAME", NULL, NULL);
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
