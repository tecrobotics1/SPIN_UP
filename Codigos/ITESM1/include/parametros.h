#ifndef PARAMETROS_H
#define PARAMETROS_H
#include <vector>

#define ROLLER_FLYWHEEL_VELOCITY 2300 //roja

#define LONG_FLYWHEEL_VELOCITY 2100 //azul

#define MEDIUM_FLYWHEEL_VELOCITY 1720  //verde 
#define SHORT_FLYWHEEL_VELOCITY 1700  //cyan 


#define STOP_FLYWHEEL_VELOCITY 0

#define ROLLER_VELOCITY_SHOOT 20  //rojo

#define HIGH_VELOCITY_SHOOT 80     //cyan 
#define MEDIUM_VELOCITY_SHOOT 65   //verde
#define LOW_VELOCITY_SHOOT 35     //azul


#define MAX_LIGHT_CONFIGURATION 4


#define ROLLER_DISTANCE_LIGHT 4
#define HIGH_DISTANCE_LIGHT 3
#define MEDIUM_DISTANCE_LIGHT 2
#define LOW_DISTANCE_LIGHT 1 

#define INDEXER_SHOOT_POSITION 180

#define DRIVER_INTAKER_VELOCITY 12000
#define DRIVER_ROLLER_VELOCITY  6000

#define EXPANSION_POSITION 360
#define TAPA_EXPANSION_POSITION 5000

#define KEEP_EXPANSION 0
#define THROW_EXPANSION 1 


//Coordena X extraida de python
extern std::vector<double> X_path;
//Coordena Y extraida de python 
extern std::vector<double> Y_path;

//Constantes de PID para DRIVE
extern std::vector<double>Drive_Constant;
//Constantes de PID para TURN 
extern std::vector<double>Turn_Constant; 

//cONSTANTES DE PID para el flywheel
extern std::vector <double>Flywheel_Constant; 

extern std::vector<double> Flywheel_Constant_primer_tiro ;
extern std::vector<double> Flywheel_Constant_segundo_tiro ;
extern std::vector<double> Flywheel_Constant_tercer_tiro ;

/*Punteros para la pasarlas como funcion y tener diferentes implementaciones en un mismo codigo*/

//Puntero para la funcion Odom_move
extern double(*fuctPtr_move_to)(double,double,double);
//Puntero para la funcion odom_move
extern double(*fuctPtr_facing_to)(double,double,double);






#endif