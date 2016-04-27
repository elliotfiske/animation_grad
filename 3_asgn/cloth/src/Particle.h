#pragma once
#ifndef __Particle__
#define __Particle__

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Shape;
class Program;
class MatrixStack;

class Particle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Particle();
	Particle(const std::shared_ptr<Shape> shape);
	virtual ~Particle();
	void tare();
	void reset();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	
	double r; // radius
	double m; // mass
	int i;  // starting index
	Eigen::Vector2d x0; // initial position
	Eigen::Vector2d v0; // initial velocity
	Eigen::Vector2d x;  // position
	Eigen::Vector2d v;  // velocity
	bool fixed;
    
    bool colliding; // Am I colliding?
    Eigen::Vector2d collision_normal; // What direction is the collision pointing?
	
private:
	const std::shared_ptr<Shape> sphere;
};

#endif
