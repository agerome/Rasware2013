#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>


int led_on;

void blink(void) {
    SetPin(PIN_BLUE, led_on);

    led_on = !led_on;
}

int main(void) {
    InitializeMCU();
    
    CallEvery(blink, 0, 1);
    
    while (1) {
        
        
    }
}
