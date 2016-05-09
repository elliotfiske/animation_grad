#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"

using namespace std;
using namespace Eigen;

Particle::Particle() :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0, 0.0),
	v(0.0, 0.0, 0.0),
	fixed(true),
    theta(0.0),
    d_theta(0.0)
{
	
}

Particle::Particle(const shared_ptr<Shape> s) :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0, 0.0),
	v(0.0, 0.0, 0.0),
	fixed(true),
    theta(0.0),
    d_theta(0.0),
	sphere(s)
{
	
}

Particle::~Particle()
{
}

void Particle::tare()
{
	x0 = x;
	v0 = v;
}

void Particle::lagrange_step(double h) {
    Vector2d J;
    J << -sin(theta), cos(theta);
    
    Matrix2d M;
    M = Matrix2d::Identity() * m;
    
    Vector2d g;
    g << 0, -9.8;
    
    double JT_M_J = J.transpose() * M * J;
    double JT_M_J_damped = J.transpose() * (M + M * h * 0.2) * J;
    
    double next_d_theta = (JT_M_J * d_theta - h * m * g.transpose() * J) /
                                          JT_M_J_damped;
    
    theta = theta + h * next_d_theta;
    d_theta = next_d_theta;
}

void Particle::reset()
{
	x = x0;
	v = v0;
}

void Particle::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	if(sphere) {
		MV->pushMatrix();
		MV->translate(Eigen::Vector3f(x(0), x(1), x(2)));
		MV->scale(r);
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, MV->topMatrix().data());
		sphere->draw(prog);
		MV->popMatrix();
	}
}

