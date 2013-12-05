#include <RASLib/inc/common.h>
#include <RASLib/inc/linesensor.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/adc.h>

// The 'main' function is the entry point of the program

tMotor *motors[2];
tI2C *bus;
tLineSensor *gls;
int main(void) {
    int error;
		float line[8];
		int x;
		int lastError = 0;
	  float speedR = 0.45; //these work at least up to 0.4
	  float speedL = 0.45;
		float kp = .15; //these might be too high? not sure
		float kd = .15;
		float PIDvalue;
		InitializeMCU();
    gls = InitializeGPIOLineSensor(PIN_B5, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_E0, PIN_C6, PIN_C7);
		motors[0] = InitializeMotor(PIN_B7, PIN_B6, true, false);
		motors[1] = InitializeMotor(PIN_F3, PIN_F2, true, false);

    
    while (1) { 
      SetMotor(motors[0],speedL);
			SetMotor(motors[1],speedR);
			
			LineSensorReadArray(gls, line);
			
			//here be filtering
			for(x = 0; x < 8; x++)
			{
				if(line[x] > 0.85)
					line[x] = 1;
				else
					line[x] = 0;
			}
			
			error = line[0] * -4 + line[1] * -2 + line[2] * -1 + line[5] + line[6] * 2 + line[7] * 4;
			if(error == 4) //far right sensor only - error = 8
				error += 4;
			if(error == -4)//far left sensor only - error = -8
				error -= 4;
			/*if(error == 0)
			{
					speedR = .35; //make sure you change the speed here too
					speedL = .35;
			}
			else
			{
				PIDvalue = error * kp + (error - lastError)/2 * kd;
				speedR -= PIDvalue;
				speedL += PIDvalue;
				if(speedR > .5) //this part prevents motor values overflowing
					speedR = .5;
				if(speedR < -.5)
					speedR = -.5;
				if(speedL > .5)
					speedL = .5;
				if(speedL < -.5)
					speedL = -.5;
				lastError = error;
			}*/
			
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
				SetMotor(motors[0],.55);
				SetMotor(motors[1],0);
			}
			else if(error > 0)
			{
				SetMotor(motors[1],.52);
				SetMotor(motors[0],0);
			}
			else
			{
				SetMotor(motors[0],.45);
				SetMotor(motors[1],.45);
			}
        
    }
}
