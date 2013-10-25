#include <RASLib/inc/common.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/time.h>

tMotor *motors[2];

// The 'main' function is the entry point of the program
int main(void) {
    InitializeMCU();
    motors[0] = InitializeMotor(PIN_B7, PIN_B6, true, false);
		motors[1] = InitializeMotor(PIN_C5, PIN_C4, true, false);
    
    while(1)
		{
	  SetMotor(motors[0], .33);
    SetMotor(motors[1], -.392);
	
		Wait(5.0);
	
		SetMotor(motors[0], 0);
    SetMotor(motors[1], 0);
			
		Wait(5.0);
		}
}
