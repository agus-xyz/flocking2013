#include <p30f6014a.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <e_uart_char.h>
#include <e_init_port.h>
#include <e_led.h>
#include <e_randb.h>
#include <e_agenda.h>
#include <e_prox.h>

#include <btcom.h>

int main() {

	/*system initialization */
	e_init_port();
	e_init_uart1();
	e_init_uart2();
	e_init_prox();
	/* range and bearing sensors */
	btcomWaitForCommand('s');

	e_start_agendas_processing();
	e_init_randb();
	/* Wait for a command comming from bluetooth */
	/* Important issue when we are transmitting data and don't want
	 * the bluetooth stack stuck for programing mode */
	e_set_body_led(1);

	/* Range is tunable by software. 
	 * 0 -> Full Range (1m. approx depending on light conditions )
	 * 255 --> No Range (0cm. approx, depending on light conditions */
	e_randb_set_range(150);

	/* At some point we tought that the board could just take
	 * data en leave the calculations for the robot.
	 * At the moment, it is better to allow the board to do the calculations */
	/*e_randb_set_calculation(ON_ROBOT);*/
	e_randb_set_calculation(ON_BOARD);

	/* Store light conditions to use them as offset for the calculation 
	 * of the range and bearing */
	e_randb_store_light_conditions();
	e_randb_set_uart_communication(1);

	/*unsigned int value;*/
	char tmp[30];

	finalDataRegister data;
	while(1)
	{
		if (e_randb_get_data_uart2(&data))
		{
			sprintf(tmp,"%02d %d %2f %d\n",data.max_sensor, data.data, data.bearing, data.range);
			btcomSendString(tmp);
		}
	}
	return 0;
}
