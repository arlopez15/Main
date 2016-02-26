void get_NW_victim(){		
    //Create something to remember which location was selected
    //then create two different paths to get back to starting
	//point (which will be Lane 2 facing yellow dropoff zone
	//at the first opening between Lane 2 and Lane 3)
	
	//Go straight 2.5 rotations
	ST.drive(20);
	motor_R_encoder.write(0);
	gyro_PID_setpoint = angle;
	while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 5) / 2)
		followGyro();
	ST.stop();
	
	ST.drive(0);
	ST.turn(-16);
	gyroAngle(angle-90);	//should be facing NW victim

	ST.drive(20);	//Move toward NW victim	

	while(millis() - last_SRF_trigger < 50);
	last_SRF_F_echo = srf_F.ping();
		Serial.println(srf_F.convert_cm(last_SRF_F_echo));
	
	if(srf_F.convert_cm(last_SRF_F_echo) < 60){	//Not sure if exactly if it will mistake the wall for the victim
		Serial.println("NW Victim");
		//go forward using photogate
		//lower arm
		arm_servo.write(ARM_DOWN);
		//open grabber
		grabber_servo.write(GRABBER_OPEN);
		//delay(500);
//		ST.drive(30);
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
		Serial.println("NNW Victim");
		
		ST.drive(0);
		ST.turn(16);
		gyroAngle(45);	//Should be facing 45 degrees to the right to run parallel with the river

		ST.drive(30);	//Drive to end of River
		ST.stop();

		ST.drive(0);
		ST.turn(-16);
		gyroAngle(-90);	//Should be facing 45 degrees to the left to run across the tip of the river

		ST.drive(10);	//Rear tires should be past the river
		ST.stop();

		ST.drive(0);
		ST.turn(-16);
		gyroAngle(angle-90);	//Should be facing left wall

		ST.drive(20);
		while(millis() - last_SRF_trigger < 50);
		last_SRF_F_echo = srf_F.ping();
			Serial.println(srf_F.convert_cm(last_SRF_F_echo));
		if(srf_F.convert_cm(last_SRF_F_echo) < 60){
			ST.stop();
			ST.drive(0);
			ST.turn(-16);
			gyroAngle(180);
		}
		else
			ST.drive(20);

		//go forward using photogate
		//lower arm
		arm_servo.write(ARM_DOWN);
		//open grabber
		grabber_servo.write(GRABBER_OPEN);
		//delay(500);
//		ST.drive(30);
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
/*	if(get_NW_victim == NW){
		ST.drive(-35);
		//for X amount of time to where it made its first 90 degree turn
		ST.stop();
		ST.drive(0);
		ST.turn(-16);
		gyroAngle(180);

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
	}
	else{	//get_NW_victim == NNW
		ST.drive(-10);
		ST.stop();
		ST.drive(0);
		ST.turn(-16);
		gyroAngle(90);

		ST.turn(0);
		ST.drive(30);
		St.stop();	//Should be lined up with Lane 2 to Lane 3 opening
		
		ST.drive(0);
		ST.turn(16);
		gyroAngle(180);	//Should be facing an obstacle

		ST.drive(20);
//want to try to swirve around the obstacle without stopping to adjust angle
		while(millis() - last_SRF_trigger < 50);
		last_SRF_F_echo = srf_F.ping();
			Serial.println(srf_F.convert_cm(last_SRF_F_echo));
		if(srf_F.convert_cm(last_SRF_F_echo) < 60){
			ST.drive(10);
			ST.turn(-10);
			ST.turn(10);
		}
		else
			ST.drive(20);
		
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
	}
*/
}
