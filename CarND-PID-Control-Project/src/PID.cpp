#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/
PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	kp = Kp;
	ki = Ki;
	kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	if(debug) {
		cout << "PID error: " << p_error << " " << i_error << " " << d_error << endl;
	}
	return ((kp * p_error) + (kd * d_error) + (ki * i_error));
}

