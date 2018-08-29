#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	state = INIT;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, _PID_STATES newState) {
	
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	
	p_error = 1.0;
	i_error = 1.0;
	d_error = 1.0;
	
	bestCte  = 1000.0;
	
	TrainingReset();
	
	state = newState;
}

void PID::UpdateError(double cte) {
	
	const float tol = 0.2; 
	
	if(state == TRAINED || TotalError() <= tol){
		state = TRAINED;
	}else{
		switch(state){
			
			case UPDATE_KP:
				std::cout << "UPDATE_KP" << std::endl;
				Kp += p_error;
				state = CHECK1_KP;
				break;
			case CHECK1_KP:
				std::cout << "CHECK1_KP" << std::endl;
				if(cte < bestCte){
					bestCte = cte;
					p_error *= 1.1;
					state = UPDATE_KI;
				}else{
					Kp -= 2.0*p_error;
					state = CHECK2_KP;
				}
				break;
			case CHECK2_KP:
				std::cout << "CHECK2_KP" << std::endl;
				if(cte < bestCte){
					bestCte = cte;
					p_error *= 1.1;
					state = UPDATE_KI;
				}else{
					Kp += p_error;
					p_error *= 0.9;
					state = UPDATE_KI;
				}
				break;
			case UPDATE_KI:
				std::cout << "UPDATE_KI" << std::endl;
				Ki += i_error;
				state = CHECK1_KI;
				break;
			case CHECK1_KI:
				std::cout << "CHECK1_KI" << std::endl;
				if(cte < bestCte){
					bestCte = cte;
					i_error *= 1.1;
					state = UPDATE_KD;
				}else{
					Ki -= 2.0*i_error;
					state = CHECK2_KI;
				}
				break;
			case CHECK2_KI:
				std::cout << "CHECK2_KI" << std::endl;
				if(cte < bestCte){
					bestCte = cte;
					i_error *= 1.1;
					state = UPDATE_KD;
				}else{
					Ki += i_error;
					i_error *= 0.9;
					state = UPDATE_KD;
				}
				break;
			case UPDATE_KD:
				std::cout << "UPDATE_KD" << std::endl;
				Kd += d_error;
				state = CHECK1_KD;
				break;
			case CHECK1_KD:
				std::cout << "CHECK1_KD" << std::endl;
				if(cte < bestCte){
					bestCte = cte;
					d_error *= 1.1;
					state = UPDATE_KP;
				}else{
					Kd -= 2.0*d_error;
					state = CHECK2_KD;
				}
				break;
			case CHECK2_KD:
				std::cout << "CHECK2_KD" << std::endl;
				if(cte < bestCte){
					bestCte = cte;
					d_error *= 1.1;
					state = UPDATE_KP;
				}else{
					Kd += d_error;
					d_error *= 0.9;
					state = UPDATE_KP;
				}	
				break;
			default: break;
			
		}
	}
}

double PID::TotalError() {
	
	return p_error + i_error + d_error;
}

double PID::Run(double cte){
	double diff_cte = cte - prev_cte;
    prev_cte = cte;
	int_cte += cte;
	double steer = -Kp * cte - Kd * diff_cte - Ki * int_cte;
	
	return steer;
}

void PID::TrainingReset(void){
	prev_cte = 0.0;
	int_cte  = 0.0;
}




