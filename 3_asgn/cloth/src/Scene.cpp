#include <iostream>

#include "Scene.h"
#include "Particle.h"
#include "Cloth.h"
#include "Shape.h"
#include "Program.h"

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
{
	// Units: meters, kilograms, seconds
	h = 1e-3;
	
	grav << 0.0, -9.8, 0.0;
	
	int rows = 5;
	int cols = 5;
	double mass = 0.1;
	double stiffness = 2e2;
	Vector2d damping(1.0, 3.0);
	Vector2d x00(-0.25, 0.5);
	Vector2d x01(0.25, 0.5);
	Vector2d x10(-0.25, 0.4);
	Vector2d x11(0.25, 0.4);
	cloth = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, stiffness, damping);
}

void Scene::init()
{
	cloth->init();
}

void Scene::tare()
{
	cloth->tare();
}

void Scene::reset()
{
	t = 0.0;
	cloth->reset();
}

void Scene::step()
{
	t += h;
	
	// Simulate the cloth
	cloth->step(h, grav, spheres);
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
	cloth->draw(MV, prog);
}
