#include "main.h"
#include "parametros.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <ostream>
#include <regex>
#include <vector>


//Erick Cuellar
//hoal amigos de youtube

void initialize() {
	Robot::reset_sensors();
	pros::delay(3000);
	std::cout<<"SENSORES OK"<<std::endl;

	screen::init();  
}





void disabled() {}


void competition_initialize() {}




void autonomous() {	
float cesar_constant = .98; 	



//Robot::start_task("PRINTING", screen::odom_stats); 
//Robot::start_task("GPS", Robot::rastreo);

         
 if(screen::autonomo==1){	
	Robot::start_task("GPS", Robot::rastreo);
	Robot::start_task("PID", Robot::Flywheel_PID_a);
		    
	// Robot::start_task("PRINTING", screen::odom_stats); 

		
	Robot::RPM=2335 * cesar_constant;  
        
	Robot::eat(200); 

	//Va por el tercer disco y encesta 
	Robot::Move_to_point(Control_move_to, 1, {-12,-5,270}, Drive_Constant, Turn_Constant, 4);
	//Robot::Move_to_point(Control_move_to, 1, {-12,-5,90+20}, Drive_Constant, Turn_Constant, 5);
	Robot::Turning(Control_move_to, 90+20, 1, Turn_Constant, 4);  
	

		//Se realiza la primera tanda
	 
	pros::delay(750);  //2000
	Robot::Shoot_normal(20);
	pros::delay(1500); 
	Robot::Shoot_normal(20); 
	pros::delay(1500); 
	Robot::Shoot_normal(20); 
	pros::delay(1000);  //1000  500 se movio antes de tiempo


	Robot::RPM=0; 
	
       
	
	//Se prepara para ir por la segunda tanda 
	Robot::eat(200); 

	Robot::RPM=2100*cesar_constant;

	//Toma el primer disco 
	Robot::Move_to_point(Control_move_to, .75, {-24.90,-6.02,220.9}, Drive_Constant, Turn_Constant,6);
	//Segundo disco
	Robot::Move_to_point(Control_move_to, .8, {-39.81,-23.12,226.80}, Drive_Constant, Turn_Constant,2.5);
	//Tercer disco
	Robot::Move_to_point(Control_move_to, .8, {-39.57,-14.12,311.42}, Drive_Constant, Turn_Constant,3);

	//Posision para segunda tiro 
	Robot::Move_to_point(Control_move_to, .8, {-41.92,-7.64,126.9-10}, Drive_Constant, Turn_Constant,3);
		
	//Segunda tanda
	pros::delay(750); //750 
	Robot::Shoot_normal(20); //20
	pros::delay(1500);  //1500
	Robot::Shoot_normal(40); //20
	pros::delay(1500); //1500
	Robot::Shoot_normal(20); //20
	pros::delay(750); 
     

	  
	//EScuadra
	Robot::Move_to_point(Control_move_to, 1, {-40.97,-28.48,129.75}, Drive_Constant, Turn_Constant,6);
	Robot::Move_to_point(Control_move_to, .8, {-17.17+5,-29.19,129.77}, Drive_Constant, Turn_Constant,9) ; //6.5
	
    //Posision para tercera tanda 
	Robot::Move_to_point(Control_move_to, .8, {-41.92,-7.64,126.9-10}, Drive_Constant, Turn_Constant,3);
		
	//tercera tanda
	pros::delay(750); 
	Robot::Shoot_normal(20); //20
	pros::delay(1500);     //1500
	Robot::Shoot_normal(20);  //20
	pros::delay(1500);  //1500
	Robot::Shoot_normal(20); //20
	pros::delay(1000); 
        

	//empezamos cuarta tanda 

	//Primer disco 
	Robot::Move_to_point(Control_move_to, .5, {-26.04,1.94,312.26}, Drive_Constant, Turn_Constant,3);
	    
	//Segundo disco
	Robot::Move_to_point(Control_move_to, .8, {-15.53,14.8,322}, Drive_Constant, Turn_Constant,2);
        
	//Velociad para la cuarta tanda
	Robot::RPM = 2410*cesar_constant; 
	//Perfilamos con el roller y agarramos tercer disco
	Robot::Move_to_point(Control_move_to, .6, {-3.69,28.35,88}, Drive_Constant, Turn_Constant,2.5);
	
	Robot::eat(0); //150 se quedaba a nada 
	pros::delay(10); 	
    
	Robot::move_chassis(150, 200); 

	pros::delay(1000); 

	Robot::Move_Roller_pos(-150, 100);
    
	pros::delay(1000); 
	Robot::move_chassis(-150, 200); 


    
  /*
	//Rotamos roller
	Robot::move_chassis(150, 200); 
	pros::delay(500); 
	Robot::move_chassis(-150, 200); 

	pros::delay(1000); 
        
	Robot::move_Indexer_velocity(30); 
	Robot::Move_to_point(Control_move_to, .9, {-3.69,28.35,95.5-2.5}, Drive_Constant, Turn_Constant,2);
	*/
  } 
  


  //Skills
  if(screen::autonomo==0){
	Robot::start_task("GPS", Robot::rastreo);
	Robot::start_task("PID", Robot::Flywheel_PID_a);
		    
	// Robot::start_task("PRINTING", screen::odom_stats); 

		
	Robot::RPM=2335 * cesar_constant;  
        
	Robot::eat(200); 

	//Va por el tercer disco y encesta 
	Robot::Move_to_point(Control_move_to, 1, {-12,-5,270}, Drive_Constant, Turn_Constant, 4);
	//Robot::Move_to_point(Control_move_to, 1, {-12,-5,90+20}, Drive_Constant, Turn_Constant, 5);
	Robot::Turning(Control_move_to, 90+20, 1, Turn_Constant, 4);  
	

		//Se realiza la primera tanda
	 
	pros::delay(750);  //2000
	Robot::Shoot_normal(20);
	pros::delay(1500); 
	Robot::Shoot_normal(20); 
	pros::delay(1500); 
	Robot::Shoot_normal(20); 
	pros::delay(1000);  //1000  500 se movio antes de tiempo


	Robot::RPM=0; 
	
       
	
	//Se prepara para ir por la segunda tanda 
	Robot::eat(200); 

	Robot::RPM=2100*cesar_constant;

	//Toma el primer disco 
	Robot::Move_to_point(Control_move_to, .75, {-24.90,-6.02,220.9}, Drive_Constant, Turn_Constant,6);
	//Segundo disco
	Robot::Move_to_point(Control_move_to, .8, {-39.81,-23.12,226.80}, Drive_Constant, Turn_Constant,2.5);
	//Tercer disco
	Robot::Move_to_point(Control_move_to, .8, {-39.57,-14.12,311.42}, Drive_Constant, Turn_Constant,3);

	//Posision para segunda tiro 
	Robot::Move_to_point(Control_move_to, .8, {-41.92,-7.64,126.9-10}, Drive_Constant, Turn_Constant,3);
		
	//Segunda tanda
	pros::delay(750); //750 
	Robot::Shoot_normal(20); //20
	pros::delay(1500);  //1500
	Robot::Shoot_normal(40); //20
	pros::delay(1500); //1500
	Robot::Shoot_normal(20); //20
	pros::delay(750); 
     

	  
	//EScuadra
	Robot::Move_to_point(Control_move_to, 1, {-40.97,-28.48,129.75}, Drive_Constant, Turn_Constant,6);
	Robot::Move_to_point(Control_move_to, .8, {-17.17+5,-29.19,129.77}, Drive_Constant, Turn_Constant,9) ; //6.5
	
    //Posision para tercera tanda 
	Robot::Move_to_point(Control_move_to, .8, {-41.92,-7.64,126.9-10}, Drive_Constant, Turn_Constant,3);
		
	//tercera tanda
	pros::delay(750); 
	Robot::Shoot_normal(20); //20
	pros::delay(1500);     //1500
	Robot::Shoot_normal(20);  //20
	pros::delay(1500);  //1500
	Robot::Shoot_normal(20); //20
	pros::delay(1000); 
        
	Robot::eat(0);
	
	Robot::Move_to_point(Control_move_to, .6, {-3.69,28.35,88}, Drive_Constant, Turn_Constant,7);
	
	 //150 se quedaba a nada 
	pros::delay(10); 	
    
	Robot::move_chassis(150, 200); 

	pros::delay(1000); 

	Robot::Move_Roller_pos(-150, 100);
    
	pros::delay(500); 
	Robot::move_chassis(-150, 200); 

	Robot::Turning(Control_move_to, 90-45, 1,Turn_Constant, 2); 
	Robot::Release_expansion(true); 




	
	
	

 


  }
 


	

}



void opcontrol() {
   
   
	Robot::start_task("get_orientation", Robot::track_orientation);
	Robot::start_task("Drive", Robot::drive);
	Robot::start_task("PID_FLYWHEEL", Robot::Flywheel_PID_a); 
	Robot::start_task("drifting", Robot::PID_drift_Alex_double); 	
   
   
 



	 
}