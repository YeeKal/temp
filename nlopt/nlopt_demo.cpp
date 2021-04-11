#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <vector>
#include <nlopt.hpp>

using namespace std;

/**
min sqrt(x2)
 
subject to x2≥0, x2≥(a1x1+b1)3, and x2≥(a2x1+b2)3

for parameters a1=2, b1=0, a2=-1, b2=1.
**/
int count=0;
double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{   
    count ++;
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct{
    double a,b;
} constrained_data;

double myConstraint(unsigned n, const double *x,double *grad, void *data){
    constrained_data *d = (constrained_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
}


int main(int argc,char** argv){
    nlopt::opt opt(nlopt::LD_MMA, 2);
    std::vector<double> lb(2);
    lb[0] = -HUGE_VAL; lb[1] = 0;
    opt.set_lower_bounds(lb);
    opt.set_min_objective(myfunc, NULL);
    constrained_data data[2] = { {2,0}, {-1,1} };
    opt.add_inequality_constraint(myConstraint, &data[0], 1e-8);
    opt.add_inequality_constraint(myConstraint, &data[1], 1e-8);
    opt.set_xtol_rel(1e-4);
    std::vector<double> x(2);
    x[0] = 1.234; x[1] = 5.678;
    double minf;

    try{
        nlopt::result result = opt.optimize(x, minf);
        printf("%d count, found minimum at f(%g,%g) = %0.10g\n",count, x[0], x[1], minf);
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    return 0;
}