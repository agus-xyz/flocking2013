/********************************
 *
 * NAME:
 * ----
 * randbEmitter/main.c
 *
 * AUTHORS:
 * -------
 * Alvaro Gutierrez -- aguti@etsit.upm.es
 * Alexandre Campo  -- acampo@ulb.ac.be
 *
 * REVISION HISTORY:
 *-----------------
 * 12/12/2008
 *
 * ADDITIONAL NOTES:
 * ----------------
 * A test for the e-randb board.
 * The code just broadcast data from 0-65535 
 */




#include <p30f6014a.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <btcom.h>
#include <e_randb.h>
#include <e_agenda.h>
#include <e_prox.h>

#include <e_uart_char.h>
#include <e_init_port.h>

#define SPACING 0
#define COHESION 1
#define HEADING 2

#define PROX_THRES 
#define COHE_THRES

#define N_ROBOTS

int main() {

	/*system initialization */
	e_init_port();
	/* Init UART1 for bluetooth */
	e_init_uart1();
	/* Init UART2 for e-randb */
	e_init_uart2();
	/* Init IR */
	e_init_prox();

	/* Wait for a command comming from bluetooth IRCOMTEST on pc directory*/
	btcomWaitForCommand('s');

	/* Start agendas processing which will take care of UART interruptions */
	e_start_agendas_processing();
	/* Init E-RANDB board */
	e_init_randb();

	/* Range is tunable by software. 
	 * 0 -> Full Range (1m. approx depending on light conditions )
	 * 255 --> No Range (0cm. approx, depending on light conditions */
	e_randb_uart_set_range(0);
	
	/* At some point we tought that the board could just take
	 * data en leave the calculations for the robot.
	 * At the moment, it is better to allow the board to do the calculations */
	e_randb_uart_set_calculation(ON_BOARD);

	/* Store light conditions to use them as offset for the calculation 
	 * of the range and bearing */
	e_randb_uart_store_light_conditions();

	//robot identifier
	int ID;
	
	//subsuncion
	int CURRENT_STATE;
	int subsuncion[3][2];

	subsuncion[0][0]=SPACING;
	subsuncion[1][0]=COHESION;
	subsuncion[2][0]=HEADING;
	for (int i=0;i<3;i++){
	  subsuncion[i][1]=0;
	}

	//proximity sensors reading
	int prox_first_reading[8];
	int prox_reading[8];
	/* Get the first reading to take ambient light */
	for(int i=0; i < 8; i++){
		first_reading[i]=e_get_prox(i);
	}
	
	//communication table
	int com_table[N_ROBOTS][3];
		
	
	/* Print on the bluetooth */
	char tmp2[50];

	

	sprintf(tmp2,"-- EMITTER --\n");
	btcomSendString(tmp2);

	/* The counter for sending */
	unsigned int data = 0;
	//unsigned long int i,j;
	
	//finalDataRegister kk;
	while(1)
	{
	  //comprobacion proximidad
	  
	  /* Get readings and substract the first reading */
	  for(int i=0; i < 8; i++){
	    sensor_reading[i] = e_get_prox(i) - first_reading[i];
	    if(sensor_reading[i] < 0) sensor_reading[i] = 0;
	  }
		

	  if(sensor_reading[0]<PROX_THRES || sensor_reading[1]<PROX_THRES || 
	     sensor_reading[2]<PROX_THRES || sensor_reading[7]<PROX_THRES ||
	     sensor_reading[6]<PROX_THRES || sensor_reading[5]<PROX_THRES){
	      subsuncion[0][1]=1;}
	  else{
	    subsuncion[0][1]=0;}

	  //comprobacion cohesion

	  int dist_average = 0;
	  int n_com = 0; //number of active communications
	  for(int i=0;i<N_ROBOTS;i++){
	    if(com_table[i][0]!=0){
	      dist_average += com_table[i][1];
	      n_com++;
	    }
	    dist_average = dist_average / (double)n_com;

	  if(dist_average>COHE_THRES){
	    subsuncion[1][1]=1;}
	  else{
	    subsuncion[1][1]=0;}
	  
	  //comprobacion heading
	  if(){
	    subsuncion[2][1]=1;}
	  else{
	    subsuncion[2][1]=0;}
	  
	  for(int i=0;i<3;i++){
	    if(subsuncion[i][1]==1){
	      CURRENT_STATE = i;
	      break;
	    }
	  }
	    switch(CURRENT_STATE){

	    case 0:

	      break;
	    case 1:


	      break;
	    case 2:


	      break;
	    }



	  //

/* Send the data through one sensor */
		//char tmp_culo[10];
		//sprintf(tmp_culo,"me ves\n");
		//e_randb_uart_store_data(0,tmp_culo);
		//e_randb_uart_send_data();
		e_randb_uart_send_all_data(data);

		/* Increase data */
		data++;
		
		/* Data to be sent must be lower than 16 bits */
		if(data==65535) data=0;
		/*for(i=0 ; i < 600 ; i++)*/
		/*for ( j = 0 ; j < 1 ; j++)*/
		/*__asm__("nop");*/
		//e_randb_get_data_uart2(&kk);
	}
	return 0;
}


