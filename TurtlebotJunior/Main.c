#include <RASLib/inc/common.h>
#include <RASLib/inc/linesensor.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/adc.h>

// The 'main' function is the entry point of the program

tMotor *motors[2];
tI2C *bus;
tLineSensor *gls;
tADC *adc[3];
int main(void) {
    int error;
		int testLine;
		int ADCLeft;
		int ADCRight;
		float line[8];
		int x;
		int lastError = 0;
	  float speedR = -0.4; //these work at least up to 0.4
	  float speedL = -0.4;
		float kp = .3; //these might be too high? not sure
		float kd = .3;
		float PIDvalue;
		int frontSensor;
		InitializeMCU();
    gls = InitializeGPIOLineSensor(PIN_B5, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_E0, PIN_C6, PIN_C7);
		motors[0] = InitializeMotor(PIN_B7, PIN_B6, true, false);
		motors[1] = InitializeMotor(PIN_F3, PIN_F2, true, false);
		adc[0] = InitializeADC(PIN_D0);
    adc[1] = InitializeADC(PIN_D1);
    adc[2] = InitializeADC(PIN_D2);
    
    while (1) {
        
      SetMotor(motors[0],speedL);
			SetMotor(motors[1],speedR);
			frontSensor = (int)(ADCRead(adc[0]) * 1000);
			
			if(frontSensor < 250)
				speedL = speedR = 0;
			else
			{
				LineSensorReadArray(gls, line);
				
				//here be filtering
				for(x = 0; x < 8; x++)
				{
					if(line[x] > 0.85)
						line[x] = 1;
					else
						line[x] = 0;
				}
				
				testLine = line[0] + line[1] + line[2] + line[3] + line[4] + line[5] + line[6] + line[7];
				if(testLine == 0)
				{
					ADCRight = (int)(ADCRead(adc[1]) * 1000);
					ADCLeft = (int)(ADCRead(adc[2]) * 1000);
					if(ADCRight > ADCLeft)
					{
						speedL += .05;
						speedR -= .05;
					}
					else if(ADCLeft < ADCRight)
					{
						speedL -= .05;
						speedR += .05;
					}
					
				}
				else
				{
					error = line[0] * -4 + line[1] * -2 + line[2] * -1 + line[5] + line[6] * 2 + line[7] * 4;
					if(error == 4) //far right sensor only - error = 8
						error += 4;
					if(error == -4)//far left sensor only - error = -8
						error -= 4;
					if(error == 0)
					{
							speedR = -.4; //make sure you change the speed here too
							speedL = -.4;
					}
					else
					{
						PIDvalue = error * kp + (error - lastError)/2 * kd;
						speedR -= PIDvalue;
						speedL += PIDvalue;
						if(speedR > 1) //this part prevents motor values overflowing
							speedR = 1;
						if(speedR < -1)
							speedR = -1;
						if(speedL > 1)
							speedL = 1;
						if(speedL < -1)
							speedL = -1;
						lastError = error;
					}
        }
    }
	}
}
