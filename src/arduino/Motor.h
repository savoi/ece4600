
#ifndef __MOTOR__
#define __MOTOR__

const bool MOTOR_DIRECTION_NORMAL = true;
const bool MOTOR_DIRECTION_REVERSE = false;

class Motor
{
	protected:
		bool direction;
		double maxVoltage;

	public:

		Motor(bool dir, double maxV)
		{
			direction = dir;
			maxVoltage = maxV;
		}

		bool getDirection();
		double getMaxVoltage();
};

#endif

