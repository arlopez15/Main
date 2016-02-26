void home_R(){
	// To end the round the robot must end in the starting location
	ST.turn(0);
	ST.drive(-30);

	do{
		while(!followSRFs(srf_FR,srf_R,true,14));	// its moving backwards and the minimum distance is 14cm
			while(millis() - last_SRF_trigger < 50);
			last_SRF_trigger = millis();
			last_SRF_L_echo = srf_L.ping();
			Serial.println(srf_L.convert_cm(last_SRF_L_echo));
	} while (srf_L.convert_cm(last_SRF_L_echo) < 36);	//find L1-L2 opening
 
	findOpening(srf_L, -25);	//Not sure if this will be needed

	angle = 0;
	ST.drive(-5);	//Not sure if this is enough to reverse into the starting location
	ST.turn(0);
	ST.stop();
}
