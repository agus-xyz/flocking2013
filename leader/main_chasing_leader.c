 #include <p30f6014a.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <btcom.h>
#include <e_randb.h>
#include <e_agenda.h>
#include <e_prox.h>
#include <e_motors.h>

#include <e_uart_char.h>
#include <e_init_port.h>

#define PROX_THRES 70
#define SPEED 800

#define PI 3.14159


int main() {
	

	/*system initialization */
	e_init_port();
	/* Init UART1 for bluetooth */
	e_init_uart1();
	/* Init UART2 for e-randb */
	e_init_uart2();
	/* Init IR */
	e_init_prox();
	//init motors
	e_init_motors();

	/* Wait for a command comming from bluetooth IRCOMTEST on pc directory*/
	//btcomWaitForCommand('s');

	/* Start agendas processing which will take care of UART interruptions */
	e_start_agendas_processing();
	/* Init E-RANDB board */
	e_init_randb();

	/* Range is tunable by software. 
	 * 0 -> Full Range (1m. approx depending on light conditions )
	 * 255 --> No Range (0cm. approx, depending on light conditions */
	e_randb_uart_set_range(128);
	
	/* At some point we tought that the board could just take
	 * data en leave the calculations for the robot.
	 * At the moment, it is better to allow the board to do the calculations */
	e_randb_uart_set_calculation(ON_BOARD);

	/* Store light conditions to use them as offset for the calculation 
	 * of the range and bearing */
	e_randb_uart_store_light_conditions();



	//proximity sensors reading
	int prox_first_reading[8];
	int prox_reading[8];
	
	/* Angles in rad for IRs starting at 0. Left direction. */
	const double prox_directions[8] = {5.9865, 5.4105, 4.7124, 3.6652, 2.6180, 1.5708, 0.8727, 0.2967};
	/* Get the first reading to take ambient light */
	int i = 0;
	for(i=0; i < 8; i++){
		prox_first_reading[i]=e_get_prox(i);
	}

	/* Print on the bluetooth */
	//char tmp2[50];

	

	//sprintf(tmp2,"-- EMITTER --\n");
	//btcomSendString(tmp2);

	/* The id for sending */
	unsigned int id = 0;
	
	while(1) {
	  e_randb_uart_send_all_data(id);

	  //comprobacion proximidad
	  
	  int maxProx = 0;
	  /* Get readings and substract the first reading */
	  for(i=0; i < 8; i++){
	    prox_reading[i] = e_get_prox(i) - prox_first_reading[i];
	    if(prox_reading[i] < 0) prox_reading[i] = 0;
	    
	    if ( prox_reading[i]>maxProx){
	      maxProx = prox_reading[i];
	    }
	  }
	  
	  /* If above a threshold */
	  if ( maxProx > PROX_THRES )
	    {
	      double vector_repelent[2] = {0.0,0.0};
	      
	      /* Calc vector Sum */
	      for (i = 0 ; i < 8; i ++ )
		{
		  vector_repelent[0] += prox_reading[i] * cos ( prox_directions[i] );
		  vector_repelent[1] += prox_reading[i] * sin ( prox_directions[i] );
		}
	      
	      /* Calc pointing angle */
	      double ang_repelent = atan2(vector_repelent[1], vector_repelent[0]);
	      /* Create repelent angle */
	      ang_repelent -= PI;
	      /* Normalize angle */
	      while ( ang_repelent > PI ) ang_repelent -= 2 * PI;
	      while ( ang_repelent < -PI ) ang_repelent += 2 * PI;
	      
	      double fCLinear = 1.0;
	      double fCAngular = 1.0;
	      double fC1 = SPEED / PI;
	      e_randb_uart_send_all_data(id);/////////////////ENVIO
	      /* Calc Linear Speed */
	      double fVLinear = SPEED * fCLinear * ( cos ( ang_repelent / 2) );
	      
	      /*Calc Angular Speed */
	      double fVAngular = ang_repelent;
	      
	      e_set_speed_left(fVLinear - fC1 * fVAngular);
	      e_set_speed_right(fVLinear + fC1 * fVAngular);
	      
	      //  if(prox_reading[0]>PROX_THRES || prox_reading[1]>PROX_THRES){
	      //	e_set_speed_left(-SPEED/2);
	      //	e_set_speed_right(SPEED/2);
	      
	      //	}else if(prox_reading[7]>PROX_THRES || prox_reading[6]>PROX_THRES){
	      //		e_set_speed_left(SPEED/2);
	      //		e_set_speed_right(-SPEED/2);
	    }
	  else{
	    e_set_speed_left(SPEED);
	    e_set_speed_right(SPEED);
	  }
	  
	  /* Send the data through one sensor */
	  
	  e_randb_uart_send_all_data(id);		
	  
	  //sprintf(tmp2,"IR: %d %d %d %d",
	  //  e_randb_uart_send_all_data(id);	  //	prox_reading[0],prox_reading[1],prox_reading[6],prox_reading[7]);
	  
	  
	  //btcomSendString(tmp2);
	  //btcomSendString("\n");
	}
	return 0;
}



