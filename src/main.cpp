#include "ch.h"
#include "hal.h"

#include "LEDs.hpp"
#include "PowerManagement.hpp"

using namespace GoodBot;


static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  int led_to_blink = 1;
  while (true)
  {
    ToggleLED(led_to_blink);
    chThdSleepMilliseconds(500);
    ToggleLED(led_to_blink);
    chThdSleepMilliseconds(500);
    
    led_to_blink++;
    if(led_to_blink > 4)
    {
        led_to_blink = 1;
    }
  }
}

int main(void)
{
    halInit();
    chSysInit();

    InitializeLEDs();
    InitializePowerManagement();

    chThdSleepMilliseconds(10000);
    SetMainPower(true);

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    while(true)
    {
        chThdSleepMilliseconds(500);
    }
}
