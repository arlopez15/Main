Skip to content
This repository  
Search
Pull requests
Issues
Gist
 @arlopez15
 Unwatch 4
  Star 0
  Fork 0 chrstphrchvz/khakis
 Code  Issues 0  Pull requests 0  Wiki  Pulse  Graphs
Branch: master Find file Copy pathkhakis/main/robot_macros.ino
b459e6d  a day ago
@jvillanueva111 jvillanueva111 lets make it go faster
2 contributors @chrstphrchvz @jvillanueva111
RawBlameHistory     909 lines (774 sloc)  24.8 KB
//blink color sensor LED once
void ledBlink(unsigned long delay_ms) {
  digitalWrite(COLOR_LED_PIN, HIGH);
  delay(delay_ms/2);
  digitalWrite(COLOR_LED_PIN, LOW);
  delay(delay_ms/2);
}

//assumes driving, gyro PID enabled (mode == AUTOMATIC)
void findOpening(NewPing& srf, int drive_speed){ 
	Serial.println("findOpening");
	unsigned long timeNow;
	unsigned long srf_reading;
	gyro_PID_setpoint = angle;
	
	//wait until opening
	Serial.println("Looking for opening...");
	ST.drive(drive_speed);
	do {
		timeNow = millis();
		if(timeNow - last_SRF_trigger >= 50){
			last_SRF_trigger = timeNow;
			srf_reading = srf.ping_cm();
			//Serial.println(srf_reading);
		}
		followGyro();
	} while(srf_reading <= 36);

	//save encoder value for opening
	Serial.print("L encoder position: ");
	int32_t encoder_opening = motor_L_encoder.read();
	Serial.println(encoder_opening);
	
	int32_t last_encoder_reading;
	Serial.println("Advancing 1/2 turn...");
	do {
		last_encoder_reading = motor_L_encoder.read();
		followGyro();
	} while(((drive_speed < 0) ? (last_encoder_reading - encoder_opening) //if going backwards
	                           : (encoder_opening - last_encoder_reading) //if going forward
	        ) < (MOTOR_COUNTS_PER_REVOLUTION / 2));
	                        
	//wait until wall
	Serial.println("Looking for wall...");
	do {
		timeNow = millis();
		if(timeNow - last_SRF_trigger >= 50){
			last_SRF_trigger = timeNow;
			srf_reading = srf.ping_cm();
			//Serial.println(srf_reading);
		}
		followGyro();
	} while (srf_reading >= 25);

	//save encoder value for wall
	Serial.print("Wall encoder position: ");
	int32_t encoder_wall = motor_L_encoder.read();
	Serial.println(encoder_wall);
	

	//reverse to middle of opening
	Serial.println("Going to opening...");
	ST.drive(-drive_speed);
	do {
		last_encoder_reading = motor_L_encoder.read();
		followGyro();
	} while((drive_speed < 0) ? last_encoder_reading > ((encoder_opening + encoder_wall)/ 2)
	                          : last_encoder_reading < ((encoder_opening + encoder_wall)/ 2));
	
	ST.stop();
}

void leaveStartingArea() {
  Serial.println("Zeroing encoders...");
  motor_L_encoder.write(0);
  motor_R_encoder.write(0);

  //initialize PID
  angle = 0;             //start with angle 0
  gyro_PID_setpoint = 0; //keep angle at 0
  gyro_PID_output = 0; //start without turning
  
  //start PID
  gyroPID.SetMode(AUTOMATIC);

  //go to opening to lane 2
  findOpening(srf_L,25);
  
  //turn left 90 degrees
  ST.drive(0);
  ST.turn(-16);
  gyroAngle(-90);

}

void L1_to_L2(){

  //go forward toward wall
  gyro_PID_setpoint = -90;
  ST.drive(30);
  do {
    if(millis() - last_SRF_trigger > 50){
      last_SRF_trigger = millis();
      last_SRF_F_echo = srf_F.ping_cm();
      Serial.println(last_SRF_F_echo);
    }
    followGyro();
  } while (last_SRF_F_echo >= 4);
  
  //turn right 45 degrees in place
  ST.drive(0);
  ST.turn(16);
  gyroAngle(-45);
  
  ST.drive(10);
  ST.turn(10);
  gyroAngle(0);

	Serial.println("Advancing 2 turns...");
	gyro_PID_setpoint = 0;
	motor_L_encoder.write(0);
	while(motor_L_encoder.read() > -MOTOR_COUNTS_PER_REVOLUTION / 2)
		followGyro(); 

	ST.stop();//now facing E city victim
  
  /*
  //go forward until wall on right
  gyro_PID_setpoint = 0;
  ST.turn(0);
  ST.drive(20);
  arm_servo.write(ARM_DOWN);
  grabber_servo.write(GRABBER_OPEN);
  while(photogateAverage() > PHOTOGATE_LOW){
    followGyro();
  }
  Serial.println("Approaching victim...");
  while(photogateAverage() < PHOTOGATE_HIGH){
    followGyro();
  }
  //stop
  ST.stop();
  
  Serial.println("Grabbing victim...");
  grabber_servo.write(GRABBER_CLOSE);
  delay(500);
  arm_servo.write(ARM_UP);
  //stop PID
  gyroPID.SetMode(MANUAL);*/
}

victim_color get_E_city(){
	//lower arm
	arm_servo.write(ARM_DOWN);
	//open grabber
	grabber_servo.write(GRABBER_OPEN);
	//delay(500);
	ST.drive(55);
	while(photogateAverage() > PHOTOGATE_LOW){
		followSRFs(srf_FR,srf_R,false,7);// its moving foward and the minimum distance is 7cm
	}
	while(photogateAverage() < PHOTOGATE_HIGH);
	
	//stop
	ST.stop();
	
	//close grabber
	grabber_servo.write(GRABBER_CLOSE);
	//wait for grabber to close
	delay(500);
	//raise arm
	arm_servo.write(ARM_UP);
	delay(1000);
	victim_color result = getColor();
	//delay(5000);//debugging: read results
	
	ST.drive(-55);
	
	//go until opening to lane 1
	do {
		followSRFs(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
	} while (srf_R.convert_cm(last_SRF_R_echo) < 30);
	
	findOpening(srf_R, -20);
	
	//turn facing lane 1
	ST.drive(0);
	ST.turn(10);
	gyroAngle(angle+90);
	ST.stop();
	
	return result;
}


void followSRF(NewPing& srf, bool is_driving_backwards){
  unsigned long timeNow = millis();
  if(timeNow - last_SRF_trigger >= 50){
    last_SRF_trigger = timeNow;
    //depending on which sensor is given, turn the robot left or right (change sign of drive power)
    int turn_sign;
    const int TURN_POWER = 2;
    if(&srf == &srf_L)
      turn_sign = is_driving_backwards ? -1 : 1;
    else if (&srf == &srf_R)
      turn_sign = is_driving_backwards ? 1 : -1;
    //else: not a valid srf???
    
    unsigned long srf_reading = srf.ping_cm();
    Serial.println(srf_reading);
    if(srf_reading < 9)
      //turn away from wall
      ST.turn((-TURN_POWER) * turn_sign);
    //else if(srf_reading == 9)
      //go straight
      //ST.turn(0);
    else //if(srf_reading > 9)
      //turn toward wall
      ST.turn(TURN_POWER * turn_sign);
  }
}

/*
Better(?) wall following algorithm using two sensors
(no idea if described somewhere online already)
Accounts for two problematic cases in old algorithm:
 -	sensor says "too far" but already facing toward wall,
 	turning more toward wall causing robot to crash
 -	sensor says "too close" but already facing away from wall,
 	turning more away causing robot to stray from path
 	(and maybe into wall on other side)
 
 New algorithm accounts for orientation of robot relative to wall;
 these cases are now handled by keeping robot straight instead of turning,
 which *should* cause robot to converge to desired distance from wall.
 
 t1 is distance of front sensor, t2 is distance of center sensor
 t2 is adjusted by T2_OFFSET_X, where
 	srf_1.ping() - (srf_2.ping() + D2_OFFSET_X) == 0
 when the robot is oriented parallel to the wall,
 and X is either L or R representing each side of the robot.
 (Theoretically, D2_OFFSET is positive if center sensor is closer
 due to standoff mount, etc., but it has been observed to still be negative,
 about ~100uS below the expected amount.
 A slight change in orientation greatly affects t1-t2,
 probably such that T2_OFFSET_X is negligible.)
 
 Table of cases:
        
        Forward, right wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing wall)   |   (facing away)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   turn away (-)   |   go straight    |
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |   go straight     |   turn toward (+)|
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
         
         
         Backwards, right wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing away)   |   (facing wall)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   go straight     |   turn toward (-)|
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |    turn away (+)  |   go straight    |
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
         
         
         
         Forward, left wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing wall)   |   (facing away)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   turn away (+)   |   go straight    |
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |   go straight     |   turn toward (-)|
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
         
         
         Backwards, left wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing away)   |   (facing wall)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   go straight     |   turn toward (+)|
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |    turn away (-)  |   go straight    |
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
 
*/
//The following declaration Must Be On One Line
//in order for the prototype to be generated.
//cf. https://github.com/arduino/arduino-builder/issues/80
bool followSRFs(NewPing& srf_front, NewPing& srf_center, bool is_driving_backwards, unsigned int target_distance){
	unsigned long timeNow = millis();
	if(timeNow - last_SRF_trigger >= 50){
		//depending on which sensor is given, turn the robot left or right
		//by changing sign of drive power
		int turn_power;
		unsigned int t2_offset;
		const int TURN_AMOUNT = 5; //adjust if needed
		unsigned int *last_SRF_front_echo, *last_SRF_center_echo;
		if      (&srf_center == &srf_L){
			turn_power = is_driving_backwards ? TURN_AMOUNT : -TURN_AMOUNT;
			t2_offset = T2_OFFSET_L;
			last_SRF_front_echo = &last_SRF_FL_echo;
			last_SRF_center_echo = &last_SRF_L_echo;
		}
		else if (&srf_center == &srf_R){
			turn_power = is_driving_backwards ? -TURN_AMOUNT : TURN_AMOUNT;
			t2_offset = T2_OFFSET_R;
			last_SRF_front_echo = &last_SRF_FR_echo;
			last_SRF_center_echo = &last_SRF_R_echo;
		}
		//else: not a valid srf???
		
		//take readings
		last_SRF_trigger = timeNow;
		*last_SRF_center_echo = srf_center.ping();
		unsigned int t2 = *last_SRF_center_echo + t2_offset;
		do{
			timeNow = millis();
		} while(timeNow - last_SRF_trigger < 50);
		last_SRF_trigger = timeNow;
		*last_SRF_front_echo = srf_front.ping();
		unsigned int& t1 = *last_SRF_front_echo;
		Serial.print("d1: ");
		Serial.println(srf_center.convert_cm(t1));
		
		unsigned int d2 = srf_center.convert_cm(t2);
		Serial.print("d2: ");
		Serial.println(d2);
		
		//compare readings
		if(!is_driving_backwards){
			if(		(d2 < target_distance)
				&&	(t1 < t2)){
				//turn away from wall
				ST.turn(-turn_power);
				Serial.println("Away");
			}
			else if((d2 > target_distance)
				&&	(t1 > t2)){
				//turn toward wall
				ST.turn(turn_power);
				Serial.println("Toward");
			}
			else{
				//go straight
				ST.turn(0);
				Serial.println("Straight");
			}
		}
		else{
			//backwards
			if(		(d2 > target_distance)
				&&	(t1 < t2)){
				//turn away from wall
				ST.turn(turn_power);
				Serial.println("Away");
			}
			else if((d2 < target_distance)
				&&	(t1 > t2)){
				//turn toward wall
				ST.turn(-turn_power);
				Serial.println("Toward");
			}
			else{
				//go straight
				ST.turn(0);
				Serial.println("Straight");
			}
		}
		return true;
	}//end if(timeNow - last_SRF_trigger >= 50)
	else
		return false;
}

void find_actual_baud(){
	long options[] = {2400,9600,19200,38400};
	int32_t starting_position = motor_L_encoder.read();
	int32_t new_position;
	int i;
	for(i = 0; i < 4; i++){
		STSerial.flush();
		STSerial.begin(options[i]);
		ST.drive(20);
		ST.turn(0);
		delay(200);
		ST.stop();
		new_position = motor_L_encoder.read();
		if(abs(new_position - starting_position) > 50){
			Serial.print("Actual baud is ");
			Serial.println(options[i]);
			return;
		}
	}
	Serial.println("Did not move");
}

void dropoff_R(){
	

   //turn left 90 degrees in place
  ST.drive(0);
  ST.turn(-16);
  gyroAngle(angle-90);
  
  ST.turn(0);
  ST.drive(55);
  
  
  
  do {
    	while(!followSRFs(srf_FR,srf_R,false,14));// its moving forward and the minimum distance is 14cm
  		
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_F_echo = srf_F.ping();
		Serial.println(srf_F.convert_cm(last_SRF_F_echo));
		
    } while (srf_F.convert_cm(last_SRF_F_echo) >= 20); //need enough room to drop arm
    
  ST.stop();
   //drop victim
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_CLOSE);
    ST.turn(0);
    ST.drive(-55);
     do {
    	while(!followSRFs(srf_FR,srf_R,true,14));// its moving backwards and the minimum distance is 14cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) < 36); //find L1-L2 opening
 
    findOpening(srf_L, -25);
    
    angle = 0;
    ST.drive(0);
    ST.turn(-16);
    gyroAngle(-90);
    ST.stop();

}
void dropoff_E_city_Y(){
	//backup to prevent hitting drop off
	motor_L_encoder.write(0);
	ST.drive(-10); //swing to avoid collision with dropoff box
	ST.turn(0);
	while(motor_L_encoder.read() < MOTOR_COUNTS_PER_REVOLUTION / 4);
	//keep turning toward yellow dropoff
	ST.drive(0);
	ST.turn(10);
	gyroAngle(180);
	ST.stop();
	
	//go further into dropoff for 1st victim
	gyro_PID_setpoint = 180;
	ST.drive(15);
	do {
    	if(millis() - last_SRF_trigger > 50){
			last_SRF_trigger = millis();
			last_SRF_F_echo = srf_F.ping_cm();
			Serial.println(last_SRF_F_echo);
		}
		followGyro();
    } while (last_SRF_F_echo >= 17); //need enough room to drop arm
    
    //drop victim
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_CLOSE);

}

void depart_from_Y_1(){
    //back up to opening
    ST.drive(-35);
	
	//go until opening to lane 3
	do {
		followSRFs(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
	} while (srf_R.convert_cm(last_SRF_R_echo) < 30);
	
	//findOpening(srf_R, -25);
	
	//turn facing lane 3
	ST.drive(-10);
	ST.turn(-10);
	gyroAngle(angle-90);
	ST.stop();
}

void depart_from_Y_2(){
    //back up to opening
    ST.drive(-50);
	
	//go until 1st opening to lane 3
	do {
		followSRFs(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
	} while (srf_R.convert_cm(last_SRF_R_echo) < 30);
	
	/* try using L1-L2 wall
	//follow gyro until L2-L3 wall is seen
	gyro_PID_setpoint = angle;
	do {
		if(millis() - last_SRF_trigger > 50) {
			last_SRF_trigger = millis();
			last_SRF_R_echo = srf_R.ping_cm();
			Serial.println(last_SRF_F_echo);
		}	
		followGyro();
	} while(srf_R.convert_cm(last_SRF_R_echo) > 25);
	*/
	
	//keep going to L2-L3 center wall
    do {
    	while(!followSRFs(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_R_echo = srf_R.ping();
		Serial.println(srf_R.convert_cm(last_SRF_R_echo));
		
    } while (srf_R.convert_cm(last_SRF_R_echo) > 30); //find L2-L3 center wall
 
	//go until 2nd opening to lane 3
   do {
    	while(!followSRFs(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_R_echo = srf_R.ping();
		Serial.println(srf_R.convert_cm(last_SRF_R_echo));
		
    } while (srf_R.convert_cm(last_SRF_R_echo) < 36); //find L2-L3 center wall
 /*
 //go until last L2-L3 wall
   do {
    	while(!followSRFs(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_R_echo = srf_R.ping();
		Serial.println(srf_R.convert_cm(last_SRF_R_echo));
		
    } while (srf_R.convert_cm(last_SRF_R_echo) > 30); //find L2-L3 center wall
 */
 
 	findOpening(srf_R,-20);
 	
	//turn facing lane 3
	ST.drive(0);
	ST.turn(10);
	gyroAngle(angle+90);
	ST.stop();
}
//victim_color get_W_city(){
void get_W_city(){
	//go backward from wall
	ST.turn(0);
	ST.drive(-20);
	gyro_PID_setpoint = angle;
	do {
		if(millis() - last_SRF_trigger > 50){
		  last_SRF_trigger = millis();
		  last_SRF_F_echo = srf_F.ping_cm();
		  Serial.println(last_SRF_F_echo);
		}
		followGyro();
	} while (last_SRF_F_echo < 20);
	
	//turn facing W city victim
	ST.turn(10);
	ST.drive(-10);
	gyroAngle(angle+90);
	
	//go until wall on right
	ST.turn(0);
	ST.drive(35);
	motor_L_encoder.write(0);
	gyro_PID_setpoint = angle;
	while( -motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 3)/2)
		followGyro();
	
	//follow wall on left until victim
	//lower arm
	arm_servo.write(ARM_DOWN);
	//open grabber
	grabber_servo.write(GRABBER_OPEN);
	//delay(500);
	while(photogateAverage() > PHOTOGATE_LOW){
		followSRFs(srf_FL,srf_L,false,8);// its moving foward and the minimum distance is 7cm
	}
	while(photogateAverage() < PHOTOGATE_HIGH);
	
	//stop
	ST.stop();
	
	//close grabber
	grabber_servo.write(GRABBER_CLOSE);
	//wait for grabber to close
	delay(500);
	//raise arm
	arm_servo.write(ARM_UP);
	delay(300);
	//delay(1000);
	//victim_color result = getColor();
	//delay(5000);//debugging: read results
	
	ST.drive(-35);
	
	//go until opening to lane 2
	do {
		followSRFs(srf_FL,srf_L,true,8);// its moving backwards and the minimum distance is 7cm
	} while (srf_L.convert_cm(last_SRF_L_echo) < 30);
	
	ST.turn(0);
	findOpening(srf_L, -25);
	
	//turn facing lane 2
	ST.drive(0);
	ST.turn(-10);
	gyroAngle(angle-45);
	ST.drive(0);
	ST.turn(-10);
	gyroAngle(angle-45);
	ST.stop();
	
	//go forward
	gyro_PID_setpoint = angle;
	ST.turn(0);
	ST.drive(20);
	do {
		if(millis() - last_SRF_trigger > 50){
		  last_SRF_trigger = millis();
		  last_SRF_F_echo = srf_F.ping_cm();
		  Serial.println(last_SRF_F_echo);
		}
		followGyro();
	} while (last_SRF_F_echo > 7);
	
	//turn facing Y drop off
	ST.drive(10);
	ST.turn(10);
	gyroAngle(angle+90);
	
	ST.turn(0);
	ST.drive(25);
	//go forward until L1-L2 opening
	  do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) < 30); //need enough room to drop arm
    
    ST.stop();
	
	//return result;
	
}

//not for E city
void dropoff_Y (){
	ST.drive(20);
	do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_F_echo = srf_F.ping();
		Serial.println(srf_F.convert_cm(last_SRF_F_echo));
		
    } while (srf_F.convert_cm(last_SRF_F_echo) >= 17); //need enough room to drop arm
    
  ST.stop();
   //drop victim
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_CLOSE);
}


//for red dropoff and return to start
void L2_W_to_L2_S(){
	//find opening on left
  findOpening(srf_L,25);
  
  //turn facing L1
  
  ST.drive(0);
  ST.turn(-16);
  gyroAngle(angle-90);
}

void L2_to_L1() {
		
	//go forward toward wall
  gyro_PID_setpoint = angle;
  ST.turn(0);
  ST.drive(35);
	
	do {
    if(millis() - last_SRF_trigger > 50){
      last_SRF_trigger = millis();
      last_SRF_F_echo = srf_F.ping_cm();
      Serial.println(last_SRF_F_echo);
    }
    followGyro();
  } while (last_SRF_F_echo >= 6);
  
  ST.stop();
}

void L2_E_to_L2_S_B(){
	//go forward to L2-L3 W opening
	ST.drive(30);
     do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) < 36);
    
    //keep going to L2-L3 center wall
    do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) > 30); //find L2-L3 center wall
 
	//rotate facing L1-L2 wall
	ST.drive(-10);
	ST.turn(10);
	gyroAngle(angle+90);
	ST.stop();
}

void L2_E_to_L2_N() {
	//go forward to L2-L3 W opening
	ST.drive(50);
     do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) < 36);
    
    //keep going to L2-L3 center wall
    do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) > 30); //find L2-L3 center wall
	
	 //keep going to second L2-L3 opening
    do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) < 36); //find L2-L3 center wall
 
 /*
 	 //keep going to last L2-L3 wall
    do {
    	while(!followSRFs(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
  		while(millis() - last_SRF_trigger < 50);
		last_SRF_trigger = millis();
		last_SRF_L_echo = srf_L.ping();
		Serial.println(srf_L.convert_cm(last_SRF_L_echo));
		
    } while (srf_L.convert_cm(last_SRF_L_echo) > 30); //find L2-L3 center wall
 */
 	
 	findOpening(srf_L,15);
 	
	//rotate facing L1-L2 wall
	ST.drive(0);
	ST.turn(-10);
	gyroAngle(angle-90);
	ST.stop();
}

void get_NE_victim(){
	//go forward 2.5 rotations before detecting obstacle
	ST.drive(20);
	motor_R_encoder.write(0);
	gyro_PID_setpoint = angle;
	while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 5) / 2)
		followGyro();
	ST.stop();
	while(millis() - last_SRF_trigger < 50);
	last_SRF_F_echo = srf_F.ping();
		Serial.println(srf_F.convert_cm(last_SRF_F_echo));
	if(srf_F.convert_cm(last_SRF_F_echo) < 60){
		Serial.println("Obstacle");
	}
	else if(srf_F.convert_cm(last_SRF_F_echo) < 113){
		Serial.println("NNE Victim");
		//go forward using photogate, wall follower//lower arm
		arm_servo.write(ARM_DOWN);
		//open grabber
		grabber_servo.write(GRABBER_OPEN);
		//delay(500);
		ST.drive(30);
		while(photogateAverage() > PHOTOGATE_LOW){
			followSRFs(srf_FR,srf_R,false,36);// its moving foward and the minimum distance is 36cm
		}
		while(photogateAverage() < PHOTOGATE_HIGH);
	
		//stop
		ST.stop();
	
		//close grabber
		grabber_servo.write(GRABBER_CLOSE);
		//wait for grabber to close
		delay(500);
		//raise arm
		arm_servo.write(ARM_UP);
		delay(1000);
		victim_color result = getColor();
		//delay(5000);//debugging: read results
	
	}
	else {
		Serial.println("ENE Victim");
	}
	
}
/*
void detect_WNW_victim() {
	//go backward from wall
	ST.turn(0);
	ST.drive(-20);
	gyro_PID_setpoint = angle;
	do {
		if(millis() - last_SRF_trigger > 50){
		  last_SRF_trigger = millis();
		  last_SRF_F_echo = srf_F.ping_cm();
		  Serial.println(last_SRF_F_echo);
		}
		followGyro();
	} while (last_SRF_F_echo < 70);
	
	//turn facing W city victim
	ST.turn(10);
	ST.drive(0);
	gyroAngle(angle+85);
	
	//sweep to detect WNW victim
	while(
}*/
Status API Training Shop Blog About Pricing
© 2016 GitHub, Inc. Terms Privacy Security Contact Help
