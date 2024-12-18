#include <VarSpeedServo.h>

// Define servo motors
#define baseServo 
#define shlServo 
#define elbServo 
#define wriServo 
#define handServo 
#define griServo 

//Arm Dimension (mm)
#define BASE_HGT 90 //base height
#define HUMERUS 100 //shoulder-to-elbow "bone"
#define ULNA 135 //elbow-to-wrist "bone"
#define GRIPPER 200 //gripper (incl.heavy duty wrist rotate mechanism) length "
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5)) //float to long conversion

//pre-calculation
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;
int servoSPeed = 10;

//ServoShield servos; //ServoShield object
VarSpeedServo servo1,servo2,servo3,servo4,servo5,servo6;

int loopCounter=0;
int pulseWidth = 6.6;
int microsecondsToDegrees;


void setup() {
  // Attach servo motors to pins
servo1.attach( baseServo, 544, 2400 );
servo2.attach( shlServo, 544, 2400 );
servo3.attach( elbServo, 544, 2400 );
servo4.attach( eriServo, 544, 2400 );
servo5.attach( handServo, 544, 2400 );
servo6.attach( griServo, 544, 2400 );
delay( 5500 );

//servos.start(); //Start the servo shield
servo_park();
delay(4000);
Serial.begin( 9600 );
Serial.println("Start");
}

void loop(){
loopCounter +=1;
set_arm( -300, 0, 100, 0 ,10); //
delay(4000);
if (loopCounter > 1) {
servo_park();
//set_arm( 0, 0, 0, 0 ,10); // park
delay(5000);
exit(0); }//pause program - hit reset to continue
}

void servo_park()
{
//servos.setposition( BAS_SERVO, 1500 );
servo1.write(90,10);
//servos.setposition( SHL_SERVO, 2100 );
servo2.write(90,10);
//servos.setposition( ELB_SERVO, 2100 );
servo3.write(90,10);
//servos.setposition( WRI_SERVO, 1800 );
servo4.write(90,10);
//servos.setposition( WRO_SERVO, 600 );
servo5.write(90,10);
//servos.setposition( GRI_SERVO, 900 );
servo6.write(80,10);
return;

}

void set_arm( float x, float y, float z, float grip_angle_d, int servoSpeed )
{
 float grip_angle_r = radians( grip_angle_d ); //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist is y coordinate for the arm */
  y = rdist;

  /* Grip offsets calculated based on grip angle */
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
  /* Wrist position */
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;
  /* Shoulder to wrist distance ( AKA sw ) */
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );
  /* s_w angle to ground */
  float a1 = atan2( wrist_z, wrist_y );
  /* s_w angle to humerus */
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );
  /* elbow angle */
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );
 /* wrist angle */
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
  /* Servo pulses */
  float bas_servopulse = 1500.0 - (( degrees( bas_angle_r )) * pulseWidth );
  float shl_servopulse = 1500.0 + (( shl_angle_d - 90.0 ) * pulseWidth );
  float elb_servopulse = 1500.0 - (( elb_angle_d - 90.0 ) * pulseWidth );
  //float wri_servopulse = 1500 + ( wri_angle_d * pulseWidth );
  //float wri_servopulse = 1500 + ( wri_angle_d * pulseWidth );
  float wri_servopulse = 1500 - ( wri_angle_d * pulseWidth );// updated 2018/2/11 by jimrd - I changed the plus to a minus - not sure how this code worked for anyone before. Could be that the elbow servo was mounted with 0 degrees facing down rather than up.
  /* Set servos */
  //servos.setposition( BAS_SERVO, ftl( bas_servopulse ));
  microsecondsToDegrees = map(ftl(bas_servopulse), 544, 2400, 0, 180);
  servo1.write(microsecondsToDegrees, servoSpeed); // use this function so that you can set servo speed //
  //servos.setposition( SHL_SERVO, ftl( shl_servopulse ));
  microsecondsToDegrees = map(ftl(shl_servopulse), 544, 2400, 0, 180);
  servo2.write(microsecondsToDegrees, servoSpeed);
  //servos.setposition( ELB_SERVO, ftl( elb_servopulse ));
  microsecondsToDegrees = map(ftl(elb_servopulse), 544, 2400, 0, 180);
  servo3.write(microsecondsToDegrees, servoSpeed);
  //servos.setposition( WRI_SERVO, ftl( wri_servopulse ));
  microsecondsToDegrees = map(ftl(wri_servopulse), 544, 2400, 0, 180);
  servo4.write(microsecondsToDegrees, servoSpeed);
}

void throw_redbox (){
//servos.setposition( BAS_SERVO, 1500 ); //change according to your box position
servo1.write(15,10);
//servos.setposition( SHL_SERVO, 2100 ); //change according to your box position
servo2.write(15,10);
//servos.setposition( ELB_SERVO, 2100 ); //change according to your box position
servo3.write(165,10);
//servos.setposition( WRI_SERVO, 1800 ); //change according to your box position
servo4.write(180,10);
//servos.setposition( WRO_SERVO, 600 ); //change according to your box position
servo5.write(180,10);
//servos.setposition( GRI_SERVO, 900 ); //change according to your box position
servo6.write(80,10);

return;
  
}
void throw_bluebox (){

//servos.setposition( BAS_SERVO, 1500 );
servo1.write(45,10);  //change according to your box position
//servos.setposition( SHL_SERVO, 2100 );
servo2.write(45,10);  //change according to your box position
//servos.setposition( ELB_SERVO, 2100 );
servo3.write(90,10);  //change according to your box position
//servos.setposition( WRI_SERVO, 1800 );
servo4.write(90,10);  //change according to your box position
//servos.setposition( WRO_SERVO, 600 );
servo5.write(180,10);  //change according to your box position
//servos.setposition( GRI_SERVO, 900 );
servo6.write(80,10);   //change according to your box position
return;

}
void through_greenbox (){
  //servos.setposition( BAS_SERVO, 1500 );
servo1.write(135,10);  //change according to your box position
//servos.setposition( SHL_SERVO, 2100 );
servo2.write(135,10);  //change according to your box position
//servos.setposition( ELB_SERVO, 2100 );
servo3.write(0,10);   //change according to your box position
//servos.setposition( WRI_SERVO, 1800 );
servo4.write(180,10); //change according to your box position
//servos.setposition( WRO_SERVO, 600 );
servo5.write(90,10);  //change according to your box position
//servos.setposition( GRI_SERVO, 900 );
servo6.write(80,10);  //change according to your box position
return;
}
void through_yellowbox (){
  //servos.setposition( BAS_SERVO, 1500 );
servo1.write(160,10);  //change according to your box position
//servos.setposition( SHL_SERVO, 2100 );
servo2.write(160,10);  //change according to your box position
//servos.setposition( ELB_SERVO, 2100 );
servo3.write(90,10);  //change according to your box position
//servos.setposition( WRI_SERVO, 1800 );
servo4.write(180,10); //change according to your box position
//servos.setposition( WRO_SERVO, 600 );
servo5.write(90,10);  //change according to your box position
//servos.setposition( GRI_SERVO, 900 );
servo6.write(80,10);  //change according to your box position
return;
}





