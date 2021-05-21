#include <stdlib.h> 
#include<math.h>
#include"MPC.h"


using namespace std;
double u1l = 5;
double u2l = 2;
const int swarm_size = 80;
const int Np = 20;
const int Nu = 20;
const int dim_bird =2*Nu;
const double inertia = 0.7298; /// arbitrary factors
const double correction_factor = 1.49618 ; 
const double length; /// to be measured




double Random(double Min, double Max)
{
    return ((double(rand()) / double(RAND_MAX)) * (Max - Min)) + Min;
}




struct bird
{
    double position[dim_bird];
    double velocity[dim_bird];
    double cost;
    double best_local_position[dim_bird];
    double best_cost;
};
class swarm
{

public:
 
    bird Bird[swarm_size];
    double bestPos[dim_bird];
    double bestCost;
    swarm(int size, int Nu);
};

swarm::swarm(int size, int Nu2)
{
    for(int i=0; i<swarm_size; i++){
        for (int j=0; j<Nu; j++){
            Bird[i].position[j] = Random(-u1l,u1l);
            Bird[i].velocity[j] = 0;
            Bird[i].cost = 1e10;
            Bird[i].best_local_position[j] = Bird[i].position[j];
            Bird[i].position[Nu + j] = Random(-u2l,u2l);
            Bird[i].best_local_position[j+Nu] = Bird[i].position[j+Nu];
            Bird[i].velocity[Nu + j] = 0;
            Bird[i].best_cost = 1e10;

        }        
    } 
}


double* MPC(double** xr, double x[3], int Nu, int itmax, double Q[3], double R[2], float to ,int sample, double r, double L) {
    swarm my_swarm(swarm_size, dim_bird);
    double xp[3][Np];
    long int cal = 1e10;
    int index  ;
    for (int i = 0; i < 3; i++)
    {
        xp[i][0]=x[i]; // from different sensors 
    }

    // PSO loop    

    ///// l'iteration max 
    for (int iteration = 0; iteration < itmax; iteration++)
    {

    for (int i = 0; i < swarm_size; i++)
    {
        double uold[]= {0,0};
        double unew[]= {0,0};
        for (int j = 0; j < Nu; j++)
        {
            my_swarm.Bird[i].position[j] += my_swarm.Bird[i].velocity[j];
        }

        // Predicition loop
        double cost =0;


        for (int k = 0; k <= Np-1; k++)
        {
            if (k <= Nu)
            {
                unew[0]= my_swarm.Bird[i].position[k];
                unew[1]= my_swarm.Bird[i].position[k+Nu];
            }
            else
            {
                unew[0]= my_swarm.Bird[i].position[Nu];
                unew[1]= my_swarm.Bird[i].position[2*Nu];
            }

            
            
                xp[0][k+1] = xp[0][k] + to*cos(xp[2][k])*unew[0];
                xp[1][k+1] = xp[1][k] + to*sin(xp[2][k])*unew[0];
                xp[2][k+1] = xp[2][k] + to*unew[1];
                // penalties
                int p1=0, p2=0;
                if (unew[0]> u1l || unew[0]< -u1l)
                    {
                        p1 =1;
                    }
                    if (unew[1]> u2l || unew[1]< -u2l)
                    {
                        p2 =1;
                    }                  
                cost += Q[0]*pow((xp[0][k]-xr[0][sample + k]),2) + Q[1]*pow((xp[1][k]-xr[1][sample + k]),2) + Q[2]*pow((xp[2][k]-xr[2][sample + k]),2) + R[0]*pow((unew[0]-uold[0]),2) + R[2]*pow((unew[1]-uold[1]),2) + cal*(p1*(pow((unew[0]-u1l),2) + p2*(pow((unew[1]-u2l),2))));
                uold[0] = unew[0];
                uold[1] = unew[1];                     
           
           
        }
        my_swarm.Bird[i].cost = cost;

       

        // update best global position 
        if (my_swarm.Bird[i].cost < my_swarm.bestCost)
        {
            my_swarm.bestCost = cost;
            index = i;
            for (int l = 0; l < 2*Nu; l++)
            {
                my_swarm.bestPos[l]= my_swarm.Bird[i].position[l];                
            }
        }
    }
     /// best personal position 
    for (int i = 0; i < swarm_size; i++)
    {
        if (my_swarm.Bird[i].cost < my_swarm.Bird[i].best_cost)
        {
            my_swarm.Bird[i].best_cost=my_swarm.Bird[i].cost;
            for ( int j = 0; j < 2*Nu; j++)
            {
                my_swarm.Bird[i].best_local_position[j]=my_swarm.Bird[i].position[j];
            }
            
        }
        
    }
    
    // update velocity : 
    for (int i = 0; i < swarm_size; i++)
    {
        for (int j = 0; j < 2*Nu; j++)
        {
            my_swarm.Bird[i].velocity[j+1]= inertia*my_swarm.Bird[i].velocity[j]   +   correction_factor * Random(0,1)*(my_swarm.Bird[i].best_local_position[j]-my_swarm.Bird[i].position[j])+correction_factor*Random(0,1)*(my_swarm.Bird[index].position[j]-my_swarm.Bird[i].position[j]);
        }
        
    }
    double vopt = my_swarm.bestPos[0], wopt= my_swarm.bestPos[Nu];
    double Vl, Vr;
    Vl = vopt/r + L/2*wopt;
    Vr = vopt/r - L/2*wopt;
    double* Vopt;
    Vopt[0]= Vl;
    Vopt[1]= Vr;
    return Vopt;

     
}
