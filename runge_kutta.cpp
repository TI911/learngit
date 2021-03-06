#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>  
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream>  
  
using namespace std;  

double dxdt(double t, double x)
{
    //double c = 0.001;
    //double r = 100;
    //double e = 10;
    //return (e-x)/(r*c);

	double x1=0, y1=0, z1=0, st=0;
	double M_PI = 3.141592;
	double tt = t;

	double radius_  = 0.10; // [m]
	double delta_ 	= 0.10; // [m]

	double a_ 		= 0.10;
	double omega_	= 4; 
	double phi_hyperbolic_ = 2*M_PI;

	x1 = (pow(a_,2)*pow(omega_,2)*pow(sinh(phi_hyperbolic_-omega_*tt),2))/pow(cosh(phi_hyperbolic_-omega_*tt),4);
	//x1 = (pow(a_,2)*pow(omega_,2)*pow(sinh(phi_hyperbolic_-omega_*tt),2))/pow(cosh(phi_hyperbolic_-omega_*tt),4);
	//y1 = pow(a_,2)/pow(cosh(phi_hyperbolic_-omega_*tt),2);
	y1 = pow(radius_+a_/cosh(phi_hyperbolic_-omega_*tt),2);
	z1 = pow(delta_,2)/(4*pow(M_PI,2));

	double j=x1 + y1 + z1;
	//st = sqrt(j);

	return sqrt(j); //st;
}

// ルンゲクッタ法(初期条件x0,  区間[t0, tn])
double runge(double x0, double t0, double tn, int n)
{
    int i;
    double x, t, h, d1, d2, d3, d4;
    x = x0;
    t = t0;
    h = (tn - t0) /n;

    // 写文件  
    ofstream outFile;
	outFile.open("runge_kutta_(r=0.1 n=0.1 a=0.1 omega=4 phi=2pi)_2.csv", ios::out); // 打开模式可省略  
 
    // 漸化式を計算
    for ( i=1; i <= n ; i++){
        t = t0 + i*h;
        d1 = dxdt(t,x);
        d2 = dxdt(t,x + d1*h*0.5);
        d3 = dxdt(t,x + d2*h*0.5);
        d4 = dxdt(t,x + d3*h);
        x += (d1 + 2 * d2 + 2 * d3 + d4)*(h/6.0); 
        printf("x(%f)=%f\n", t, x);
		outFile << t << ',' << x << endl;
    }			
    outFile.close(); 
	return x;
}
 

int main(void)
{
    runge(0, 0, 20, 2000); 

	system("pause");
    return 1;
}