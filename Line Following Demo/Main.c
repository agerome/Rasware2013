#include <RASLib/inc/common.h>
#include <RASLib/inc/linesensor.h>
#include <RASLib/inc/motor.h>

// The 'main' function is the entry point of the program
tMotor *motors[2];
tI2C *bus;
tLineSensor *ls;

int main(void) {
		int error;
		float line[8];
		int x;
		//int lastError = 0;
	  //float speedR = .35;
	  //float speedL = .35;
		InitializeMCU();
		bus = InitializeI2C(PIN_B3, PIN_B2);
    ls = InitializeLineSensor(bus, 0);
		motors[0] = InitializeMotor(PIN_B7, PIN_B6, true, false);
		motors[1] = InitializeMotor(PIN_C5, PIN_C4, true, false);
    // Initialization code can go here
    
    while (1) {
      
			
			//SetMotor(motors[0],speedR);
			//SetMotor(motors[1],speedL);
			
			 
        // Runtime code can go here
			
			//float kp = .1;
			//float kd = .1;
			//float PIDvalue;
			LineSensorReadArray(ls, line);
			
			//here be filtering
			for(x = 0; x < 8; x++)
			{
				if(line[x] > 0.85)
					line[x] = 1;
				else
					line[x] = 0;
			}
			
			error = line[0] * -3 + line[1] * -2 + line[2] * -1 + line[5] + line[6] * 2 + line[7] * 3;
			
			/*PIDvalue = error * kp + (error - lastError)/2 * kd;
			speedR += PIDvalue;
			SetMotor(motors[0],speedR);*/
			
			/*switch(error)
			{
				case -5:
					speedL = speedR - .1;
					break;
				case -3:
					speedL = speedR - .067;
					break;
				case -1:
					speedL = speedR - .033;
					break;
				case 1:
					speedL = speedR + .033;
					break;
				case 3:
					speedL = speedR + .067;
					break;
				case 5:
					speedL = speedR + .1;
					break;
				default:
					speedL = speedR;
					break;
			}*/
			if(error < 0)
			{
				SetMotor(motors[0],.35);
				SetMotor(motors[1],0);
			}
			else if(error > 0)
			{
				SetMotor(motors[1],.42);
				SetMotor(motors[0],0);
			}
			else
			{
				SetMotor(motors[0],.35);
				SetMotor(motors[1],.35);
			}
			
		
		}
}
