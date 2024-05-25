#include <Adafruit_MCP4725.h>
#include <Adafruit_Sensor.h>
#include <PID_v2.h>
#include <Wire.h>
Adafruit_MCP4725 dac;


//########################################
// designates the rov pulling velocity

double vel=0.1;// m/s
//########################################

double reference = vel*477.7; //477.7 include the gearing ratio and drum radius
double SpeedInRPM = 0;
float Kp=0.005, Ki=0.01, Kd=0;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);


#define ClockPin 3 // Must be pin 2 or 3

// The encoder has 1024 pulses
// note that 58593.75 = (60 seonds * 1.000.000 microseconds)microseconds in a minute / 1024 pulses in 1 revolution)
#define Multiplier 58593.7 // don't forget a decimal place to make this number a floating point number
volatile long count = 0; // Volatile means that the variable has the ability to change unexpectetly, hence disrupts complier optimisation
volatile long EncoderCounter = 0;
volatile int32_t dTime; // Delt in time
volatile bool DataPinVal;

/* ##################      Void Setup ###########################*/
void setup(void) { // put your setup code here, to run once:

  Serial.begin(115200);
  dac.begin(0x60); // MCP4725A1 address is 0x60 (default)
  dac.setVoltage(0, false); // startvalue at 0 voltage which is equal to 0 velocity on the motor. 
  
  pinMode(ClockPin, INPUT);
  //  pinMode(DataPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ClockPin), onPin2CHANGECallBackFunction, RISING);

  
  myPID.SetOutputLimits(0, 5);
  myPID.Start(SpeedInRPM,  // input
              0,                      // current output
              reference);                   // setpoint

    //WE SET COMMENTS IN THIS 3 LINE STO GET RID OF THE ENTER CONDITION
  //Serial.println("The operation is paused, please turn on the switches before pressing enter");
  //while (Serial.available() ==0){} // Awaits an input from the serial monitor.
  //delay(10);
}


/* ##################      FUNCTION: Encoder Data  #####################*/
void onPin2CHANGECallBackFunction() {
  static uint32_t lTime; // Saved Last Time of Last Pulse
  uint32_t cTime; // Current Time
  cTime = micros(); // Store the time for RPM Calculations

  /*
    // Encoder Code
      DataPinVal = digitalRead(DataPin);
    // We know pin 2 just went high to trigger the interrupt
    // depending on direction the data pin will either be high or low
      EncoderCounter += (DataPinVal) ? 1 : -1; 
  */
  // calculate the DeltaT between pulses
  dTime = cTime - lTime;
  lTime = cTime;

}

/* ##################      VOID LOOP ###########################*/
void loop() {
  //  long Counter;
  // bool DataPinValue
  
  float DeltaTime;
  double SpeedInRPM = 0;
double input, output;
    
  // Serial print is slow so only use it when you need to (10 times a second)
  static unsigned long SpamTimer;
  if ( (unsigned long)(millis() - SpamTimer) >= (100)) {
    SpamTimer = millis();
    noInterrupts ();
    // Because when the interrupt occurs the EncoderCounter and SpeedInRPM could be interrupted while they
    // are being used we need to say hold for a split second while we copy these values down. This doesn't keep the
    // interrupt from occurring it just slightly delays it while we maneuver values.
    // if we don't do this we could be interrupted in the middle of copying a value and the result get a corrupted value.
    //  Counter = EncoderCounter;
    //  DataPinValue = DataPinVal;
    DeltaTime = dTime;
    dTime = 0; // if no pulses occure in the next 100 miliseconds then we must assume that the motor has stopped this allows a speed of zero to occure 
    interrupts ();
    //   SpeedInRPM = Multiplier / ((DataPinValue) ? DeltaTime: (-1 * DeltaTime)); // Calculate the RPM Switch DeltaT to either positive or negative to represent Forward or reverse RPM
    SpeedInRPM = Multiplier / DeltaTime; // Calculate the RPM Switch DeltaT to either positive or negative to represent Forward or reverse RPM
    if(DeltaTime == 0){SpeedInRPM = 0.0;}
    
   input = SpeedInRPM;
  output = myPID.Run(input);   
   
     
      dac.setVoltage(output*4095/5,false); // computes the output value from the PID (as an integer) to the MCP4725.
      myPID.Compute();
      
    //   Serial.print(Counter );
    //   Serial.print("\t");
    //Convertion of X [RPM] to [m/s] -> 1rev/min* 2*pi*radius[m]/1 rev *1[min]/60[s]
    // 1 rev is 0.396m distance we have measured 
    
    Serial.print(SpeedInRPM/477.7,3);
    //Serial.print(" m/s");
    Serial.println();



  }
 


  //Get sensor values and set equal to start position
  //calculate the error and PID + set voltage for MCP 4725

}
