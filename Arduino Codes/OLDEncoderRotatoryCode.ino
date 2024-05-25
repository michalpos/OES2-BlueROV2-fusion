// Pins
const int hallSensor1Pin = 2; 
const int hallSensor2Pin = 3; 

// Variables
volatile int hallSensor1State = 0;
volatile int hallSensor2State = 0;
volatile long encoderCount = 0;
volatile int encoderDirection = 0;

// Resolution of the encoder (pulses per revolution)
const int pulsesPerRevolution = 192;

void setup() {
  // Initialize
  Serial.begin(9600);
  // Set inputs
  pinMode(hallSensor1Pin, INPUT);
  pinMode(hallSensor2Pin, INPUT);
 // Enable resistors
  digitalWrite(hallSensor1Pin, HIGH);
  digitalWrite(hallSensor2Pin, HIGH);
 // Make interrupts
  attachInterrupt(digitalPinToInterrupt(hallSensor1Pin), hallSensor1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallSensor2Pin), hallSensor2ISR, CHANGE);
}

void loop() {
  // Calculate angular position from encoder count
  float angularPosition = (float)encoderCount / pulsesPerRevolution * 360;

  // Print the angular position and direction
  //Serial.print("Angular Position: ");
  Serial.print(angularPosition);
  Serial.println();
  //Serial.print(" degrees, Direction: ");
  //if (encoderDirection == 1) {
    //Serial.println("Forward");
  //} else if (encoderDirection == -1) {
    //Serial.println("Backward");
  //} else {
    //Serial.println("No movement");
  //}

  delay(20); //delay
}

// Interrupt 1
void hallSensor1ISR() {
  // Update encoder
  if (digitalRead(hallSensor1Pin) == digitalRead(hallSensor2Pin)) {
    encoderDirection = 1; // Forward
  } else {
    encoderDirection = -1; // Backward
  }

  // Increment encoder count
  encoderCount += encoderDirection;
}

// Interrupt 2
void hallSensor2ISR() {
  // Update encoder direction based on Hall-effect sensor 2 state
  if (digitalRead(hallSensor1Pin) == digitalRead(hallSensor2Pin)) {
    encoderDirection = -1; // Backward
  } else {
    encoderDirection = 1; // Forward
  }

  // Increment encoder count
  encoderCount += encoderDirection;
}
