
//WARNING: not constrained to safe range
void arm_test(){
	Serial.print("Enter 3 digit angle: ");
	while(Serial.available() < 3);
	int16_t servo_angle = Serial.parseInt();
	Serial.println(servo_angle);
	//servo_angle = constrain(servo_angle);
	arm_servo.write(servo_angle);
}
