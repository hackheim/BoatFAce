#include <SoftwareSerial.h>

#include <SPI.h>


#include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        7  //14    
#define PS2_CMD        6  //15
#define PS2_SEL        5  //16
#define PS2_CLK        4  //17

SoftwareSerial motors(11,12);

#define pressures   true
#define rumble      true

PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;

void setup(){
  Serial.begin(57600);
  motors.begin(57600);
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it


  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
	if (pressures)
	  Serial.println("true ");
	else
	  Serial.println("false");
	Serial.print("rumble = ");
	if (rumble)
	  Serial.println("true)");
	else
	  Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
   }
}

void loop() {
  if(error == 1) //skip loop if no controller found
    return; 
  
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }   

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      Serial.println("Square just released");     

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      /*Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); */


      //code under is from https://www.impulseadventure.com/elec/robot-differential-steering.html
      // OUTPUTS
      int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
      int     nMotMixR;           // Motor (right) mixed output           (-128..+127)
      
      // CONFIG
      // - fPivYLimt  : The threshold at which the pivot action starts
      //                This threshold is measured in units on the Y-axis
      //                away from the X-axis (Y=0). A greater value will assign
      //                more of the joystick's range to pivot actions.
      //                Allowable range: (0..+127)
      float fPivYLimit = 32.0;
            
      // TEMP VARIABLES
      float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
      float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
      int     nPivSpeed;      // Pivot Speed                          (-128..+127)
      float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
      int nJoyX = -(ps2x.Analog(PSS_LX)-128);
      int nJoyY = -(ps2x.Analog(PSS_LY)-127);
      
      // Calculate Drive Turn output due to Joystick X input
      if (nJoyY >= 0) {
        // Forward
        nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
        nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
      } else {
        // Reverse
        nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
        nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
      }
      
      // Scale Drive output due to Joystick Y input (throttle)
      nMotPremixL = nMotPremixL * nJoyY/128.0;
      nMotPremixR = nMotPremixR * nJoyY/128.0;
      
      // Now calculate pivot amount
      // - Strength of pivot (nPivSpeed) based on Joystick X input
      // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
      nPivSpeed = nJoyX;
      fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);
      
      // Calculate final mix of Drive and Pivot
      nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
      nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);

      float rate;
      if(ps2x.Button(PSB_L3))
        rate = 1.0;//BÆÆMM!!
      else
        rate = 0.5;
      
      int leftMotor = nMotMixL*rate;
      int rightMotor = nMotMixR*rate;

      Serial.print(leftMotor, DEC); 
      Serial.print(",");
      Serial.println(rightMotor, DEC); 

      motors.print("L");
      motors.println(leftMotor);
      motors.print("R");
      motors.println(rightMotor);

    }     
  delay(50);  
}
