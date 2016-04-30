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
            if(i == 0 && (j == 0 || j == cols-1)) {
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
    
    Vector2d real_grav;
    real_grav << grav(0), grav(1);
   
   A_trips.clear();
   SparseMatrix<double> A_sparse(n, n);
   
   // Set up our delicious right-hand vector.
   VectorXd b;
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

void Cloth::solve_with_mosek(double h) {
    
    int num_entries_M = A_trips.size();
    int NUMVAR = f.rows();
    
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
    
    MSKint32t     qsubi[num_entries_M];
    MSKint32t     qsubj[num_entries_M];
    double        qval[num_entries_M];
    
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
//            if ( r == MSK_RES_OK )
//                r = MSK_appendcons(task,NUMVAR);
            
            /* Append 'NUMVAR' variables.
             The variables will initially be fixed at zero (x=0). */
            if ( r == MSK_RES_OK )
                r = MSK_appendvars(task,NUMVAR);
            
            /* Optionally add a constant term to the objective. */
//            if ( r ==MSK_RES_OK )
//                r = MSK_putcfix(task,0.0);
            
            for(j=0; j<f.rows() && r == MSK_RES_OK; ++j)
            {
                /* Set the linear term c_j in the objective.*/
                if(r == MSK_RES_OK)
                    r = MSK_putcj(task,j,f(j));
                
                /* Set the bounds on variable j.
                 blx[j] <= x_j <= bux[j] */
                if(r == MSK_RES_OK)
                    r = MSK_putvarbound(task,
                                        j,           /* Index of variable.*/
                                        MSK_BK_LO,        //bkx[j],      /* Bound key.*/
                                        -100.0,              //blx[j],      /* Numerical value of lower bound.*/
                                        +MSK_INFINITY);   //bux[j]);     /* Numerical value of upper bound.*/

                /* Input column j of A */
//                if(r == MSK_RES_OK)
//                    r = MSK_putacol(task,
//                                    j,                 /* Variable (column) index.*/
//                                    aptre[j]-aptrb[j], /* Number of non-zeros in column j.*/
//                                    asub+aptrb[j],     /* Pointer to row indexes of column j.*/
//                                    aval+aptrb[j]);    /* Pointer to Values of column j.*/
                
            }
            
            /* Set the bounds on constraints.
             for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
//            for(i=0; i<NUMCON && r==MSK_RES_OK; ++i)
//                r = MSK_putconbound(task,
//                                    i,           /* Index of constraint.*/
//                                    bkc[i],      /* Bound key.*/
//                                    blc[i],      /* Numerical value of lower bound.*/
//                                    buc[i]);     /* Numerical value of upper bound.*/
            
            if ( r==MSK_RES_OK )
            {
                /*
                 * The lower triangular part of the Q
                 * matrix in the objective is specified.
                 */
                
                for (int ndx = 0; ndx < A_trips.size (); ndx++) {
                    if (A_trips[ndx].row() >= A_trips[ndx].col()) {
                        qsubi[ndx] = A_trips[ndx].row();
                        qsubj[ndx] = A_trips[ndx].col();
                        qval[ndx] += 0.5 * A_trips[ndx].value();
                    }
                }
                
//                qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = 2.0;
//                qsubi[1] = 1;   qsubj[1] = 1;  qval[1] = 0.2;
//                qsubi[2] = 2;   qsubj[2] = 0;  qval[2] = -1.0;
//                qsubi[3] = 2;   qsubj[3] = 2;  qval[3] = 2.0;
                
                /* Input the Q for the objective. */
                
                r = MSK_putqobj(task,num_entries_M,qsubi,qsubj,qval);
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
//                MSK_solutionsummary (task,MSK_STREAM_MSG);
                
                if ( r==MSK_RES_OK )
                {
                    MSKsolstae solsta;
//                    int j;
                    
                    MSK_getsolsta (task,MSK_SOL_ITR,&solsta);
                    
                    switch(solsta)
                    {
                        case MSK_SOL_STA_OPTIMAL:
                        case MSK_SOL_STA_NEAR_OPTIMAL:
                            MSK_getxx(task,
                                      MSK_SOL_ITR,    /* Request the interior solution. */
                                      xx);
                            
                            printf("Optimal primal solution\n");
//                            for(j=0; j<NUMVAR; ++j)
//                                printf("x[%d]: %e\n",j,xx[j]);
                            
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

    
    
    // Set each particles' new velocity
    for (int ndx = 0; ndx < particles.size(); ndx++) {
        shared_ptr<Particle> p = particles[ndx];
        
        int particle_ndx = p->i;
        if (particle_ndx != -1) {
            p->v(0) = xx[particle_ndx];
            p->v(1) = xx[particle_ndx + 1];
//            p->v = result_v.block<2, 1>(particle_ndx, 0);
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
