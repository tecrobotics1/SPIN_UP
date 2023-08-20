#ifndef  PID_HPP
#define PID_HPP

/*
Header de PID, incluye todas las variables necesarias para elaborar un controlador PID
*/

//Estructura de PID
typedef struct{

   	float kp;         
  	float ki;         								
	float kd;    

	float integral_raw;  
	float last_error;  

	float zonaintegralactiva; 
	float integralpowerlimit; 
 
	float error; 

	float proporcion; 
	float integral;
	float derivada;

	int maximo;
	
	float finalpower;            							
 
}PID_Control;

//Objetos para los compensadores que utilizamos 

//Objeto para el compensador de posicion
extern PID_Control DRIVE;
//Objeto para el compensador de orientacion 
extern PID_Control TURN;

extern PID_Control FLYWHEEL; 

extern float Get_Proporcion (PID_Control Compensator);

extern float Get_Integral_raw (PID_Control Compensator);

extern float Get_Integral(PID_Control Compensator); 

extern float Get_Derivada(PID_Control Compensator);

extern float Get_Pid_Output(PID_Control Compensator, float potencia); 

#endif