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

#define SPACING 0
#define COHESION 1
#define HEADING 2

#define PROX_THRES 70
#define SPEED 1000

#define PI 3.14159


void setAngularVelocity(double ang,double force){

double fCLinear, fCAngular, fC1,fVLinear,fVAngular;
  	
 while ( ang > PI ) {ang -= 2 * PI;}
 while ( ang < -PI ) {ang += 2 * PI;}

      fCLinear = 1.0;
      fCAngular = 1.0;
      fC1 = force * SPEED / PI;
      
      /* Calc Linear Speed */
      fVLinear = SPEED * fCLinear * ( cos ( ang / 2) );
      
      /*Calc Angular Speed */
      fVAngular = ang;
      
      e_set_speed_left(fVLinear - fC1 * fVAngular);
      e_set_speed_right(fVLinear + fC1 * fVAngular);
}

/* The id for sending */
unsigned int id = 1;

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
  btcomWaitForCommand('s');
  
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
  e_randb_set_uart_communication(1);
  
  finalDataRegister data;
  
  //subsuncion
  int CURRENT_STATE;
  int subsuncion[2][2];
  int debug_var = 0;
  subsuncion[0][0]=SPACING;
  subsuncion[1][0]=COHESION;
  
  int i;
  for (i=0;i<2;i++){
    subsuncion[i][1]=0;
  }
  
  //proximity sensors reading
  int prox_first_reading[8];
  int prox_reading[8];

  /* Angles in rad for IRs starting at 0. Left direction. */
  const double prox_directions[8] = {5.9865, 5.4105, 4.7124, 3.6652, 2.6180, 1.5708, 0.8727, 0.2967};
	


  /* Get the first reading to take ambient light */
  for(i=0; i < 8; i++){
    prox_first_reading[i]=e_get_prox(i);
  }
  
  /* Print on the bluetooth */
  char tmp2[50];
  // sprintf(tmp2,"-- CHASER --\n");
  //btcomSendString(tmp2);

  char tmp[30];  
  
  double vector_repelent[2] = {0.0,0.0};
  double ang_repelent;
  double ang_comunicacion = 0;    
  
  
  
  while(1) {
    //comprobacion proximidad
    //    int maxProx = 0;
    /* Get readings and substract the first reading */
    //for(i=0; i < 8; i++){
    //  prox_reading[i] = e_get_prox(i) - prox_first_reading[i];
    //  if(prox_reading[i] < 0) {prox_reading[i] = 0; }
    //  if ( prox_reading[i]>maxProx){
    //	maxProx = prox_reading[i];
    //  }
    //}
    int maxProx = 0;
    /* Get readings and substract the first reading */
    for(i=0; i < 8; i++){
      prox_reading[i] = e_get_prox(i) - prox_first_reading[i];
      if(prox_reading[i] < 0) prox_reading[i] = 0;
      
      if ( prox_reading[i]>maxProx){
	maxProx = prox_reading[i];
      }
    }
  



    if(maxProx > PROX_THRES){
      subsuncion[0][1]=1;  // Collission
    }
    else{  // Chasing
      subsuncion[0][1]=0;
    }
      
    CURRENT_STATE = 1; //by default. chasing
    for(i=0;i<2;i++){
      if(subsuncion[i][1]==1){
	CURRENT_STATE = i;
	break;
      }
    }
    
    
    if (e_randb_get_data_uart2(&data)){
      sprintf(tmp2,"ID %d \n",data.data);
      
      if((data.data==0) && ((data.bearing > -2*PI) && (data.bearing < 2*PI))){
	
	ang_comunicacion = data.bearing;
      }
    }
    //  else {
      // ang_comunicacion = 0;
    // sprintf(tmp2,"Angulo CERO \n");
    //}
    sprintf(tmp2,"Angulo de comunicacion %f \n",ang_comunicacion);
    btcomSendString(tmp2);
    
    switch(CURRENT_STATE){
    case 0: //Collission
      
      //	  sprintf(tmp2,"-- COLISION max= %d --\n",maxProx);
      //	btcomSendString(tmp2);
      
      /* Calc vector Sum */
      //      for (i = 0 ; i < 8; i ++ )
      //{
      //  vector_repelent[0] += prox_reading[i] * cos ( prox_directions[i] );
      //  vector_repelent[1] += prox_reading[i] * sin ( prox_directions[i] );
      //}
      //
      /* Calc pointing angle */
      //ang_repelent = atan2(vector_repelent[1], vector_repelent[0]);
      /* Create repelent angle */
      //ang_repelent -= PI;
      
      
      vector_repelent[0] = 0.0;
      vector_repelent[1] = 0.0;      

      /* Calc vector Sum */
      for (i = 0 ; i < 8; i ++ ) {
	vector_repelent[0] += prox_reading[i] * cos ( prox_directions[i] );
	vector_repelent[1] += prox_reading[i] * sin ( prox_directions[i] );
      }
      
      /* Calc pointing angle */
      ang_repelent = atan2(vector_repelent[1], vector_repelent[0]);
      ang_repelent -= PI;      


      //calculate and set velocity
      setAngularVelocity(ang_repelent,1);

      break; // Case 0
      
    case 1:	// Chasing 
      // sprintf(tmp2,"-- PERSIGUIENDO--\n");
      //		btcomSendString(tmp2);

		
	if(ang_comunicacion > 0.4 || ang_comunicacion < -0.4){ // Chasing
      		setAngularVelocity(ang_comunicacion,2);
		sprintf(tmp2,"-- PERSIGUIENDO--\n");
      		btcomSendString(tmp2);
	}
	else { // Walk 
	  //sprintf(tmp2,"-- RECTO--\n");
	  //	btcomSendString(tmp2);
	  e_set_speed_left(SPEED);
	  e_set_speed_right(SPEED);
	}		
    	break; // Case 1
    } // End switch

	/* Send the data through one sensor */
		
    e_randb_uart_send_all_data(id);	

   
    //if (e_randb_get_data_uart2(&data)){
   // sprintf(tmp,"%d %02f %d %2f %d\n",debug_var, data.bearing, data.range);
   // btcomSendString(tmp);
    /* Send the data through one sensor */
    
    //e_randb_uart_send_all_data(data);
  }
    
  return 0;
}



