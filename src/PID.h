#ifndef PID_H
#define PID_H

class PID {
	
	
	
	double prev_cte;
	double int_cte;
	
	double bestCte;
	
	
public:

	enum _PID_STATES
    {
      INIT,
	  UPDATE_KP,
	  CHECK1_KP,
	  CHECK2_KP,
	  UPDATE_KI,
	  CHECK1_KI,
	  CHECK2_KI,
	  UPDATE_KD,
	  CHECK1_KD,
	  CHECK2_KD,
	  TRAINED
    };
	
	PID::_PID_STATES state;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, _PID_STATES newState);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  //
  double Run(double cte);
  
  void TrainingReset(void);
};

#endif /* PID_H */
