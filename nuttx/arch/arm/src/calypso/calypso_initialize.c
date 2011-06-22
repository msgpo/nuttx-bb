#include <nuttx/arch.h>

void up_initialize(void)
{
        (void)calypso_armio();
	(void)calypso_keypad();
}
