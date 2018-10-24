/*  L-BFGS-B is released under the "New BSD License" (aka "Modified BSD License" */
/*  or "3-clause license") */
/*  Please read attached file License.txt */

#ifndef lbfgsb_solver
#define lbfgsb_solver

#include <Eigen/Core>

typedef long int integer;
typedef long int ftnlen;
typedef long int logical;

namespace lbfgsb {
    struct Problem
    {
        virtual double value(double *x) = 0;
        virtual void gradient(double *x, double *g) = 0;
    };
    
    struct Solver
    {
        /* Local variables */
        double f, g[1024];
        double l[1024],u[1024];
        integer m = 5;
        integer n;
        double x[1024], t1, t2, wa[43251];
        integer nbd[1024], iwa[3072];
        
        integer taskValue;
        integer* task=&taskValue; /* must initialize !! */
        /*      http://stackoverflow.com/a/11278093/269192 */
        
        integer csaveValue;
        integer* csave = &csaveValue;
        double dsave[29];
        integer isave[44];
        logical lsave[4];
        
        double factr = 1e7; // stopping criteria
        double pgtol = 1e-5; // stopping criteria
        integer iprint = -1; // print out status every iprint-iterations ( <0 surpress varbose)
        
        int maxIter = 50;
        
        inline void setDim(integer _n){ n = _n; }
        
        void setBoxConstriant(const Eigen::VectorXd& lb,const Eigen::VectorXd& ub);
        
        void solve(Problem &problem,Eigen::VectorXd& x0);
    };

}

#endif
