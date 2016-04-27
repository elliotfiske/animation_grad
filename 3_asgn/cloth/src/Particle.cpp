#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"

using namespace std;

Particle::Particle() :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0),
	v(0.0, 0.0),
	fixed(true)
{
	
}

Particle::Particle(const shared_ptr<Shape> s) :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0),
	v(0.0, 0.0),
	fixed(true),
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

void Particle::reset()
{
	x = x0;
	v = v0;
}

void Particle::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
}
