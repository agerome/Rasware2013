#include <RASLib/inc/common.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>

int main(void) {
	tADC *adc;
	int ADCValue;
	tMotor *motors[2];
	InitializeMCU();
	adc = InitializeADC(PIN_D0);
	motors[0] = InitializeMotor(PIN_B7, PIN_B6, true, false);
	motors[1] = InitializeMotor(PIN_C5, PIN_C4, true, false);
    
    
    while (1) {
			
			ADCValue = (int)(ADCRead(adc) * 1000);
      if(ADCValue > 500)
			{
				SetMotor(motors[0],-.65);
				SetMotor(motors[1],0);
			}
			else if(ADCValue < 400)
			{
				SetMotor(motors[0],0);
				SetMotor(motors[1],.65);
			}
			else
			{
				SetMotor(motors[0],-.45);
				SetMotor(motors[1],.45);
			}
        
    }
}
