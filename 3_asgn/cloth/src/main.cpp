#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif
#include <thread>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

#include "GLSL.h"
#include "Program.h"
#include "Camera.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Scene.h"

#include "mosek_man.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

bool keyToggles[256] = {false}; // only for English keyboards!

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

shared_ptr<Camera> camera;
shared_ptr<Program> prog;
shared_ptr<Program> progSimple;
shared_ptr<Scene> scene;

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

static void char_callback(GLFWwindow *window, unsigned int key)
{
	keyToggles[key] = !keyToggles[key];
	switch(key) {
		case 'h':
			scene->step();
			break;
		case 'r':
			scene->reset();
			break;
	}
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if(state == GLFW_PRESS) {
		camera->mouseMoved(xmouse, ymouse);
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
	// Enable alpha blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	progSimple = make_shared<Program>();
	progSimple->setShaderNames(RESOURCE_DIR + "simple_vert.glsl", RESOURCE_DIR + "simple_frag.glsl");
	progSimple->setVerbose(false); // Set this to true when debugging.
	progSimple->init();
	progSimple->addUniform("P");
	progSimple->addUniform("MV");
	
	prog = make_shared<Program>();
	prog->setVerbose(true); // Set this to true when debugging.
	prog->setShaderNames(RESOURCE_DIR + "phong_vert.glsl", RESOURCE_DIR + "phong_frag.glsl");
	prog->init();
	prog->addUniform("P");
	prog->addUniform("MV");
	prog->addUniform("kdFront");
	prog->addUniform("kdBack");
	prog->addAttribute("vertPos");
	prog->addAttribute("vertNor");
	//prog->addAttribute("vertTex");
	
	camera = make_shared<Camera>();

	scene = make_shared<Scene>();
	scene->load(RESOURCE_DIR);
	scene->tare();
	scene->init();
	
	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
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
	float x0 = -0.5f;
	float x1 = 0.5f;
	float z0 = -0.5f;
	float z1 = 0.5f;
	int gridSize = 10;
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for(int i = 1; i < gridSize; ++i) {
		if(i == gridSize/2) {
			glColor3f(0.1f, 0.1f, 0.1f);
		} else {
			glColor3f(0.8f, 0.8f, 0.8f);
		}
		float x = x0 + i / (float)gridSize * (x1 - x0);
		glVertex3f(x, 0.0f, z0);
		glVertex3f(x, 0.0f, z1);
	}
	for(int i = 1; i < gridSize; ++i) {
		if(i == gridSize/2) {
			glColor3f(0.1f, 0.1f, 0.1f);
		} else {
			glColor3f(0.8f, 0.8f, 0.8f);
		}
		float z = z0 + i / (float)gridSize * (z1 - z0);
		glVertex3f(x0, 0.0f, z);
		glVertex3f(x1, 0.0f, z);
	}
	glEnd();
	glColor3f(0.4f, 0.4f, 0.4f);
	glBegin(GL_LINE_LOOP);
	glVertex3f(x0, 0.0f, z0);
	glVertex3f(x1, 0.0f, z0);
	glVertex3f(x1, 0.0f, z1);
	glVertex3f(x0, 0.0f, z1);
	glEnd();
	progSimple->unbind();

	// Draw scene
	prog->bind();
	glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, P->topMatrix().data());
	MV->pushMatrix();
	scene->draw(MV, prog);
	MV->popMatrix();
	prog->unbind();
	
	//////////////////////////////////////////////////////
	// Cleanup
	//////////////////////////////////////////////////////
	
	// Pop stacks
	MV->popMatrix();
	P->popMatrix();
	
	GLSL::checkError(GET_FILE_LINE);
}

void stepperFunc()
{
	while(true) {
		if(keyToggles[(unsigned)' ']) {
			scene->step();
		}
		this_thread::sleep_for(chrono::microseconds(1));
	}
}


/* This function prints log output from MOSEK to the terminal. */
static void MSKAPI printstr(void *handle,
                            MSKCONST char str[])
{
   printf("%s",str);
} /* printstr */

#define NUMCON 1   /* Number of constraints.             */
#define NUMVAR 3   /* Number of variables.               */
#define NUMANZ 3   /* Number of non-zeros in A.           */
#define NUMQNZ 4   /* Number of non-zeros in Q.           */

int do_thing()
{
    
    
    
    double        f_squiggle[]   = {0.0,-1.0,0.0};
    
    MSKboundkeye  bkc[] = {MSK_BK_LO};
    double        blc[] = {1.0};
    double        buc[] = {+MSK_INFINITY};
    
    MSKboundkeye  bkx[] = {MSK_BK_LO,
        MSK_BK_LO,
        MSK_BK_LO};
    double        blx[] = {0.0,
        0.0,
        0.0};
    double        bux[] = {+MSK_INFINITY,
        +MSK_INFINITY,
        +MSK_INFINITY};
    
    MSKint32t     aptrb[] = {0,   1,   2},
    aptre[] = {1,   2,   3},
    asub[]  = {0,   0,   0};
    double        aval[]  = {1.0, 1.0, 1.0};
    
    MSKint32t     qsubi[NUMQNZ];
    MSKint32t     qsubj[NUMQNZ];
    double        qval[NUMQNZ];
    
    MSKint32t     i,j;
    double        xx[NUMVAR];
    
    MSKenv_t      env = NULL;
    MSKtask_t     task = NULL;
    MSKrescodee   r;
    
    /* Create the mosek environment. */
    r = MSK_makeenv(&env,NULL);
    
    if ( r==MSK_RES_OK )
    {
        /* Create the optimization task. */
        r = MSK_maketask(env,NUMCON,NUMVAR,&task);
        
        if ( r==MSK_RES_OK )
        {
            
            // Grab clock time BEFORE doing magic multiplication
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            
            r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
            
            /* Append 'NUMCON' empty constraints.
             The constraints will initially have no bounds. */
            if ( r == MSK_RES_OK )
                r = MSK_appendcons(task,NUMCON);
            
            /* Append 'NUMVAR' variables.
             The variables will initially be fixed at zero (x=0). */
            if ( r == MSK_RES_OK )
                r = MSK_appendvars(task,NUMVAR);
            
            /* Optionally add a constant term to the objective. */
            if ( r ==MSK_RES_OK )
                r = MSK_putcfix(task,0.0);
            for(j=0; j<NUMVAR && r == MSK_RES_OK; ++j)
            {
                /* Set the linear term c_j in the objective.*/
                if(r == MSK_RES_OK)
                    r = MSK_putcj(task,j,f_squiggle[j]);
                
                /* Set the bounds on variable j.
                 blx[j] <= x_j <= bux[j] */
                if(r == MSK_RES_OK)
                    r = MSK_putvarbound(task,
                                        j,           /* Index of variable.*/
                                        bkx[j],      /* Bound key.*/
                                        blx[j],      /* Numerical value of lower bound.*/
                                        bux[j]);     /* Numerical value of upper bound.*/
                
                /* Input column j of A */
                if(r == MSK_RES_OK)
                    r = MSK_putacol(task,
                                    j,                 /* Variable (column) index.*/
                                    aptre[j]-aptrb[j], /* Number of non-zeros in column j.*/
                                    asub+aptrb[j],     /* Pointer to row indexes of column j.*/
                                    aval+aptrb[j]);    /* Pointer to Values of column j.*/
                
            }
            
            /* Set the bounds on constraints.
             for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
            for(i=0; i<NUMCON && r==MSK_RES_OK; ++i)
                r = MSK_putconbound(task,
                                    i,           /* Index of constraint.*/
                                    bkc[i],      /* Bound key.*/
                                    blc[i],      /* Numerical value of lower bound.*/
                                    buc[i]);     /* Numerical value of upper bound.*/
            
            if ( r==MSK_RES_OK )
            {
                /*
                 * The lower triangular part of the Q
                 * matrix in the objective is specified.
                 */
                
                qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = 2.0;
                qsubi[1] = 1;   qsubj[1] = 1;  qval[1] = 0.2;
                qsubi[2] = 2;   qsubj[2] = 0;  qval[2] = -1.0;
                qsubi[3] = 2;   qsubj[3] = 2;  qval[3] = 2.0;
                
                /* Input the Q for the objective. */
                
                r = MSK_putqobj(task,NUMQNZ,qsubi,qsubj,qval);
            }
            
            if ( r==MSK_RES_OK )
            {
                MSKrescodee trmcode;
                
                /* Run optimizer */
                r = MSK_optimizetrm(task,&trmcode);
                
                
                
                // Grab clock time AFTER doing tons of pointless math
                high_resolution_clock::time_point t2 = high_resolution_clock::now();
                
                // Print duration!
                auto duration = duration_cast<microseconds>( t2 - t1 ).count();
                cout << "Results: " << duration << " µs" << endl;
                
                /* Print a summary containing information
                 about the solution for debugging purposes*/
                MSK_solutionsummary (task,MSK_STREAM_MSG);
                
                if ( r==MSK_RES_OK )
                {
                    MSKsolstae solsta;
                    int j;
                    
                    MSK_getsolsta (task,MSK_SOL_ITR,&solsta);
                    
                    switch(solsta)
                    {
                        case MSK_SOL_STA_OPTIMAL:
                        case MSK_SOL_STA_NEAR_OPTIMAL:
                            MSK_getxx(task,
                                      MSK_SOL_ITR,    /* Request the interior solution. */
                                      xx);
                            
                            printf("Optimal primal solution\n");
                            for(j=0; j<NUMVAR; ++j)
                                printf("x[%d]: %e\n",j,xx[j]);
                            
                            break;
                        case MSK_SOL_STA_DUAL_INFEAS_CER:
                        case MSK_SOL_STA_PRIM_INFEAS_CER:
                        case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
                        case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
                            printf("Primal or dual infeasibility certificate found.\n");
                            break;
                            
                        case MSK_SOL_STA_UNKNOWN: 
                            printf("The status of the solution could not be determined.\n"); 
                            break; 
                        default: 
                            printf("Other solution status."); 
                            break; 
                    } 
                } 
                else 
                { 
                    printf("Error while optimizing.\n"); 
                } 
            } 
            
            if (r != MSK_RES_OK) 
            { 
                /* In case of an error print error code and description. */       
                char symname[MSK_MAX_STR_LEN]; 
                char desc[MSK_MAX_STR_LEN]; 
                
                printf("An error occurred while optimizing.\n");      
                MSK_getcodedesc (r, 
                                 symname, 
                                 desc); 
                printf("Error %s - '%s'\n",symname,desc); 
            } 
        } 
        MSK_deletetask(&task); 
    } 
    MSK_deleteenv(&env);
    
    
    
    return (r);
}



int main(int argc, char **argv)
{
   do_thing();
   
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
	// Start simulation thread.
	thread stepperThread(stepperFunc);
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
	stepperThread.detach();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
