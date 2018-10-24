#include "solver.h"
#include "lbfgsb.h"
#include <iostream>

namespace lbfgsb{

void Solver::setBoxConstriant(const Eigen::VectorXd& lb,const Eigen::VectorXd& ub)
{
    assert(lb.size() == n && ub.size() == n);
    // set upper/lower bound
    for(int i = 0; i < n; ++i)
    {
        l[i] = lb[i];
        u[i] = ub[i];
        nbd[i] = 2;
    }
}

void Solver::solve(Problem &problem,Eigen::VectorXd& x0)
{
    // initialize the variables
    
    for(int i = 0; i < n; ++i)
    {
        x[i] = x0[i];
    }
    
    /*     We start the iteration by initializing task. */
    
    *task = (int)START;
    
    do{
        setulb(&n, &m, x, l, u, nbd, &f, g, &factr, &pgtol, wa, iwa, task, &iprint, csave, lsave, isave, dsave);
        
        if ( IS_FG(*task) ) {
            
            // compute objective function
            f = problem.value(x);
            
            // compute gradient
            problem.gradient(x,g);
        
            /*          go back to the minimization routine. */
        }
        
        if( isave[33] > maxIter){
            *task = STOP_ITER;
            setulb(&n, &m, x, l, u, nbd, &f, g, &factr, &pgtol, wa, iwa, task, &iprint, csave, lsave, isave, dsave);
        }
        
    }while((*task==NEW_X || IS_FG(*task)));
    
    for(int i = 0; i < n; ++i)
    {
        x0[i] = x[i];
    }
}

}