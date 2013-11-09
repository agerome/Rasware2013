#include <RASLib/inc/common.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/time.h>

// The 'main' function is the entry point of the program
int main(void) {
    tMotor *motors[2];
		InitializeMCU();
		motors[0] = InitializeMotor(PIN_B7, PIN_B6, true, false);
		motors[1] = InitializeMotor(PIN_C5, PIN_C4, true, false);
    
    while (1) {
        SetMotor(motors[0],-1);
				SetMotor(motors[1],.91);
        Wait([AMOUNT OF TIME IT TAKES TO GO FORWARD 4 FEET AKA 1.219 METERS THIS IS IMPORTANT);
				SetMotor(motors[0],-1);
				SetMotor(motors[1],-.91);
				Wait([AMOUNT OF TIME IT TAKES TO TURN 90 DEGREES AKA PI/2 RADIANS THIS IS ALSO IMPORTANT]);
    }
}
