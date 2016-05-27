#include <iostream>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

#include "mosek_man.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector2d x0 = p0->x;
	Vector2d x1 = p1->x;
	Vector2d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}


Cloth::Cloth(int rows, int cols,
			 const Vector2d &x00,
			 const Vector2d &x01,
			 const Vector2d &x10,
			 const Vector2d &x11,
			 double mass,
			 double stiffness,
			 const Vector2d &damping)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->rows = rows;
	this->cols = cols;
	this->damping = damping;
	
	// Create particles
	n = 0;
	double r = 0.02; // Used for collisions
	int nVerts = rows*cols;
	for(int i = 0; i < rows; ++i) {
		double u = i / (rows - 1.0);
		Vector2d x0 = (1 - u)*x00 + u*x10;
		Vector2d x1 = (1 - u)*x01 + u*x11;
		for(int j = 0; j < cols; ++j) {
			double v = j / (cols - 1.0);
			Vector2d x = (1 - v)*x0 + v*x1;
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
			p->x = x;
			p->v << 0.0, 0.0;
			p->m = mass/(nVerts);
			// Pin two particles
         if(i == 0 && (j == 0)) {// || j == cols-1)) {
				p->fixed = true;
				p->i = -1;
			} else {
				p->fixed = false;
				p->i = n;
				n += 2;
			}
		}
	}
   
   all_springs_for_particle.resize(particles.size());
   positive_springs_for_particle.resize(particles.size());
   negative_springs_for_particle.resize(particles.size());
	
	// Create x springs
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + 1;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
         
         all_springs_for_particle[k0].push_back(springs.back());
         all_springs_for_particle[k1].push_back(springs.back());
         positive_springs_for_particle[k0].push_back(springs.back());
         negative_springs_for_particle[k1].push_back(springs.back());
		}
	}
	
	// Create y springs
	for(int j = 0; j < cols; ++j) {
		for(int i = 0; i < rows-1; ++i) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
         
         all_springs_for_particle[k0].push_back(springs.back());
         all_springs_for_particle[k1].push_back(springs.back());
         positive_springs_for_particle[k0].push_back(springs.back());
         negative_springs_for_particle[k1].push_back(springs.back());
		}
	}
	
	// Create shear springs
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k00 = i*cols + j;
			int k10 = k00 + 1;
			int k01 = k00 + cols;
			int k11 = k01 + 1;
         springs.push_back(createSpring(particles[k00], particles[k11], stiffness));
         all_springs_for_particle[k00].push_back(springs.back());
         all_springs_for_particle[k11].push_back(springs.back());
         positive_springs_for_particle[k00].push_back(springs.back());
         negative_springs_for_particle[k11].push_back(springs.back());
         
			springs.push_back(createSpring(particles[k10], particles[k01], stiffness));
         all_springs_for_particle[k10].push_back(springs.back());
         all_springs_for_particle[k01].push_back(springs.back());
         positive_springs_for_particle[k10].push_back(springs.back());
         negative_springs_for_particle[k01].push_back(springs.back());
		}
	}

	// Build system matrices and vectors
	M.resize(n,n);
	K.resize(n,n);
	v.resize(n);
	f.resize(n);
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();
	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}
	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}



void Cloth::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
    A_trips.clear();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector2d x = particles[k]->x;
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = 0;
		}
	}
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     /|\
			// u0 /_|_\ u1
			//    \ | /
			//     \|/
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector2d x = particles[k]->x;
			Vector2d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector2d nor(0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
//				c = dx0.cross(dx1);
//				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
//				c = dx0.cross(dx1);
//				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
//				c = dx0.cross(dx1);
//				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
//				c = dx0.cross(dx1);
//				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = 0;
		}
	}
}

// Iterate through every particle and check if it's colliding with something
void Cloth::check_for_collisions() {
    // Do floor
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->colliding = false;
        if (particles[i]->x(1) < 0) {
            particles[i]->colliding = true;
            particles[i]->collision_normal = Vector2d(0.0, 1.0);
        }
    }
}


// Iterates through a 2x2 matrix and puts all the non-zero
//  entries into the list o' triplets.
//
// offset_x and offset_y determine where the matrix is in the super big matrix. ITS LATE OK
void Cloth::convert_2x2_mat_to_trips(const Eigen::Matrix2d &mat, int offset_row, int offset_col) {
   for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
         double curr_num = mat(i, j);
         
         if (curr_num != 0) {
            A_trips.push_back(Trip(offset_row + i, offset_col + j, curr_num));
         }
      }
   }
}

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();
   b.setZero();
    
    Vector2d real_grav;
    real_grav << grav(0), grav(1);
   
   A_trips.clear();
   SparseMatrix<double> A_sparse(n, n);
   
   // Set up our delicious right-hand vector.
   b.resize(n);
   
   // Loop through every single particle
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      Matrix2d mini_M = p->m * Matrix2d::Identity();
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         vector<shared_ptr<Spring> > curr_springs = all_springs_for_particle[ndx];
         vector<shared_ptr<Spring> > negative_curr_springs = negative_springs_for_particle[ndx];
         vector<shared_ptr<Spring> > positive_curr_springs = positive_springs_for_particle[ndx];
         
         Matrix2d diagonal_k;
         diagonal_k.setZero();
         
         // Check all of the springs attached to the CURRENT PARTICLE
         for (int s_ndx = 0; s_ndx < curr_springs.size(); s_ndx++) {
            shared_ptr<Spring> s = curr_springs[s_ndx];
            Vector2d dx = s->p1->x - s->p0->x;
            double l = dx.norm();
            
            double l_L_l = (l - s->L)/l;
            double scalar_thing = (l_L_l * dx.transpose()*dx);
            // "ks" is mini ks block between the current particle and the other particle on the end of the current spring
            Matrix2d ks = s->E/(l*l) * (
                                        (1 - l_L_l) * dx*dx.transpose() +
                                        scalar_thing * Matrix2d::Identity());
            
            ks *= damping[1] * h*h;
            
            // We do the diagonal blocks of K separately, cuz they're the only ones that depend
            // on more than 1 spring.
            diagonal_k += ks;
            
            // Tricky part! Here we fill in the entries that look like this:
            //
            //
            //   x- - -     and nothing else!
            //   |
            //   |
            //   |
            //
            //   They're all equal to -ks.
            if (s->p0->i != -1 && s->p1->i != -1) {
               if (s->p0->i > p->i) {
                  convert_2x2_mat_to_trips(-ks, s->p0->i, p->i);
                  convert_2x2_mat_to_trips(-ks, p->i, s->p0->i);
               }
               
               if (s->p1->i > p->i) {
                  convert_2x2_mat_to_trips(-ks, s->p1->i, p->i);
                  convert_2x2_mat_to_trips(-ks, p->i, s->p1->i);
               }
            }
         }
         // end looping through all the silly springs
         
         // Take the diagonal K value and add the Mass to it
         diagonal_k += mini_M + mini_M * damping[0] * h;
         convert_2x2_mat_to_trips(diagonal_k, p->i, p->i);
         
         // Set the Mv component of b
         Vector2d mv = mini_M * p->v;
         b.block<2,1>(particle_ndx, 0) = mv + p->m * real_grav * h;
         
         // calculate the rest of b with the forces (easy)
         for (int p_ndx = 0; p_ndx < positive_curr_springs.size(); p_ndx++) {
            shared_ptr<Spring> s = positive_curr_springs[p_ndx];
            Vector2d dx = s->p1->x - s->p0->x;
            double l = dx.norm();
            Vector2d fs = s->E * (l - s->L) * dx / l;
            fs *= h;
            
            if (s->p0->i != -1) {
               b.block<2, 1>(particle_ndx, 0) += fs;
            }
         }
         
         // dont look at this code its bad ok
         
         for (int n_ndx = 0; n_ndx < negative_curr_springs.size(); n_ndx++) {
            shared_ptr<Spring> s = negative_curr_springs[n_ndx];
            Vector2d dx = s->p1->x - s->p0->x;
            double l = dx.norm();
            Vector2d fs = s->E * (l - s->L) * dx / l;
            fs *= h;
            
            if (s->p1->i != -1) {
               b.block<2, 1>(particle_ndx, 0) += -fs;
            }
         }
      }
   }
    
    check_for_collisions();
    
    solve_with_mosek(h);
	
	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, MV->topMatrix().data());
	int h_pos = p->getAttribute("vertPos");
	GLSL::enableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("vertNor");
	GLSL::enableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
//	int h_tex = p->getAttribute("vertTex");
//	GLSL::enableVertexAttribArray(h_tex);
//	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
//	glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	//GLSL::disableVertexAttribArray(h_tex);
	GLSL::disableVertexAttribArray(h_nor);
	GLSL::disableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}

#define NUMCON 0   /* Number of constraints.             */
//#define NUMVAR 3   /* Number of variables.               */
#define NUMANZ 3   /* Number of non-zeros in A.           */

/* This function prints log output from MOSEK to the terminal. */
static void MSKAPI printstr(void *handle,
                            MSKCONST char str[])
{
    printf("%s",str);
} /* printstr */


void got_error(MSKrescodee r) {
   /* In case of an error print error code and description. */
   char symname[MSK_MAX_STR_LEN];
   char desc[MSK_MAX_STR_LEN];
   
   printf("An error occurred while optimizing.\n");
   MSK_getcodedesc (r,
                    symname,
                    desc);
   printf("Error %s - '%s'\n",symname,desc);
}


void Cloth::solve_with_mosek(double h) {
   
   int num_entries_M = A_trips.size();
   int num_vars = n; // Equal to the number of particles * 3
   
   vector<double> result;
   result.resize(num_vars);
   
   
   MSKenv_t env = NULL;
   MSKrescodee r = MSK_RES_OK;
   MSKtask_t task = NULL;
   
   int num_constraints = 0; // TODO: this will be equal to the # of collisions
   
   r = MSK_makeenv(&env, NULL);
   if (r != MSK_RES_OK) { printf("ERROR: making env\n"); got_error(r); }
   
   r = MSK_maketask(env, num_constraints, num_vars, &task);
   if (r != MSK_RES_OK) { printf("ERROR: making task\n"); got_error(r); }
   
   r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
   if (r != MSK_RES_OK) { printf("ERROR: hooking up printstr\n"); got_error(r); }
   
   r = MSK_appendcons(task, num_constraints);
   if (r != MSK_RES_OK) { printf("ERROR: appending constraints\n"); got_error(r); }
   
   r = MSK_appendvars(task, num_vars);
   if (r != MSK_RES_OK) { printf("ERROR: appending vars\n"); got_error(r); }
   
   for (int j = 0; j < num_vars; j++) {
      r = MSK_putcj(task, j, -b[j]);
      if (r != MSK_RES_OK) { printf("ERROR: inputting b-vector\n"); got_error(r); }
      
      r = MSK_putvarbound(task, j, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
      if (r != MSK_RES_OK) { printf("ERROR: setting variable bound\n"); got_error(r); }
      
      // TODO: input columns for constraints here
   }
   
   // TODO: set bounds on constrains here (greater than zero hahahahaha)
   
   // Input Q matrix
   vector<MSKint32t> qsubi;
   vector<MSKint32t> qsubj;
   vector<double>     qval;
   
   qsubi.resize(num_entries_M);
   qsubj.resize(num_entries_M);
   qval.resize(num_entries_M);
   
   int data_ndx = 0; // This is where the actual data is getting plonked into the array.
                     //   It differs from ndx by the number of upper-triangular values we're getting.
   
   for (int ndx = 0; ndx < A_trips.size (); ndx++) {
      if (A_trips[ndx].row() >= A_trips[ndx].col()) {
         qsubi[data_ndx] = (MSKint32t) A_trips[ndx].row();
         qsubj[data_ndx] = (MSKint32t) A_trips[ndx].col();
         qval[data_ndx]  = A_trips[ndx].value();
         
         data_ndx++;
      }
   }
   
   r = MSK_putqobj(task, data_ndx, &qsubi[0], &qsubj[0], &qval[0]);
   if (r != MSK_RES_OK) { printf("ERROR: inputting Q\n"); got_error(r); }
   
   MSKrescodee trmcode;
   
   r = MSK_optimizetrm(task, &trmcode);
   if (r != MSK_RES_OK) { printf("ERROR: solving probbo\n"); got_error(r); }
   
   MSKsolstae solsta;
   MSK_getsolsta (task, MSK_SOL_ITR, &solsta);
   switch (solsta) {
      case MSK_SOL_STA_OPTIMAL:
      case MSK_SOL_STA_NEAR_OPTIMAL:
         MSK_getxx(task, MSK_SOL_ITR, &result[0]);
         break;
      default:
         printf("Couldn't find solution! got something weird: %d\n", solsta);
   }
   
   MSK_deletetask(&task);
   MSK_deleteenv(&env);
   
   // Set each particles' new velocity
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         p->v(0) = result[particle_ndx];
         p->v(1) = result[particle_ndx + 1];
         //            p->v = result_v.block<2, 1>(particle_ndx, 0);
         printf("X vel: %f Y vel: %f\n", result[particle_ndx], result[particle_ndx + 1]);
      }
   }
   
   // Set each particles' new position
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         p->x += h * p->v;
      }
   }
}
