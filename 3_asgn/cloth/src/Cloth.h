#pragma once
#ifndef __Cloth__
#define __Cloth__

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class Spring;
class MatrixStack;
class Program;

typedef Eigen::Triplet<double> Trip;

class Cloth
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Cloth(int rows, int cols,
		  const Eigen::Vector2d &x00,
		  const Eigen::Vector2d &x01,
		  const Eigen::Vector2d &x10,
		  const Eigen::Vector2d &x11,
		  double mass,
		  double stiffness,
		  const Eigen::Vector2d &damping);
	virtual ~Cloth();
	
	void tare();
	void reset();
	void updatePosNor();
	void step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres);
	
	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	
private:
	int rows;
	int cols;
	int n;
	Eigen::Vector2d damping;
	std::vector< std::shared_ptr<Particle> > particles;
	std::vector< std::shared_ptr<Spring> > springs;
	
	Eigen::VectorXd v;
	Eigen::VectorXd f;
	Eigen::MatrixXd M;
	Eigen::MatrixXd K;
	
	std::vector<unsigned int> eleBuf;
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
    
    void convert_2x2_mat_to_trips(const Eigen::Matrix2d &mat, int offset_row, int offset_col);
    
    // At each index N, contains a list of the springs for the Nth particle.
    std::vector< std::vector< std::shared_ptr<Spring> > > all_springs_for_particle;
    std::vector< std::vector< std::shared_ptr<Spring> > > negative_springs_for_particle;
    std::vector< std::vector< std::shared_ptr<Spring> > > positive_springs_for_particle;
    
    // My list of entries in the sparse matrix
    std::vector<Trip> A_trips;

    std::vector<Particle> colliding_particles;
    void check_for_collisions();
    
    void solve_with_mosek(double h);
    
	unsigned eleBufID;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
};

#endif
