// Andrey Zhuravlev
// email: v.azhure@gmail.com

#ifndef Motor_h
#define Motor_h

#define MOTOR_SPEED_OFF 0

// Motor class
class Motor
{
  private:
	// Enabled flag
	bool bEnabled;
	// Motor control pin
	int control_pin;
	// PWM motor pin
	int pwm_pin;
	// Precision in TICKS
	int32_t tolerance;
	// Encoder ticks per unit
	double unitsPerCount;

  public:
	// Constructor
	// [cp]: motor direction control pin
	// [pwmp]: minimum motor speed (pwm)
	// [upc]: units per ticks count
	// [s]: default motor speed (pwm)
	Motor(int cp, int pwmp, double upc, int32_t tol)
	{
		bEnabled = true;
		control_pin = cp;
		pwm_pin = pwmp;
		unitsPerCount = upc;
		tolerance = tol;

		pinMode(control_pin, OUTPUT);

		Stop();
	}

	inline int32_t GetTolerance() { return tolerance; }

	inline bool GetEnabled() { return bEnabled; }
	inline void SetEnabled(bool enable)
	{
		if (enable != bEnabled)
		{
			if (!enable)
				Stop();
			bEnabled = enable;
		}
	}

	// Convert double value to ticks
	inline int32_t DBL_TO_ENC(double val)
	{
		return (int32_t)(val / unitsPerCount);
	}

	// Convert ticks to double
	inline double ENC_TO_DBL(double val)
	{
		return val * unitsPerCount;
	}

	// Enable moving in forward direction with defined speed
	// speed: [0..255]
	inline void MoveForvard(int s)
	{
		if (bEnabled)
		{
			digitalWrite(control_pin, HIGH);
			analogWrite(pwm_pin, s);
		}
	}

	// Enable moving in backward direction with defined speed
	// speed: [0..255]
	inline void MoveBack(int s)
	{
		if (bEnabled)
		{
			digitalWrite(control_pin, LOW);
			analogWrite(pwm_pin, s);
		}
	}

	// Disable moving (set speed to zero)
	inline void Stop()
	{
		analogWrite(pwm_pin, MOTOR_SPEED_OFF);
	}

	inline void Move(int32_t pwm)
	{
		if (bEnabled)
		{
			digitalWrite(control_pin, pwm < 0 ? LOW : HIGH);
			analogWrite(pwm_pin, abs(pwm));
		}
	}

	// Main position function.
	// posDest: destination in encoder ticks
	// posCur: current encoder ticks
	inline void UpdatePosition(int32_t posDest, int32_t posCur, int32_t pwm)
	{
		int32_t delta = abs(posCur - posDest);

		if (delta <= tolerance)
		{
			Stop(); // In position
		}
		else
		{
			if (posDest > posCur)
			{
				MoveForvard(pwm);
			}
			else if (posDest < posCur)
			{
				MoveBack(pwm);
			}
		}
	}
};

#endif Motor_h
