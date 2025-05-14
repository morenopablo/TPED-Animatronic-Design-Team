#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // This is the 'maximum' pulse length count (out of 4096)
#define EYEMIN 220  // Adjust this to your servo's minimum pulse length
#define EYEMAX 480  // Adjust this to your servo's maximum pulse length
#define EYEMID ((EYEMIN + EYEMAX) / 2)  // Calculate the middle position
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define NUMSERVOS 4 // number of servos
#define EYESERVO 7
#define EYEBROWRIGHT 8
#define EYEBROWLEFT 9
#define LEDPIN 7
bool lastButtonState = HIGH;

const int buttonPin = 2; // An LED pin to show when the function is triggered (optional)


void customFunction() {
  //moves the four servos after 7 back and forth
  int servonum[4] = {7, 8, 9, 10}; 
  Serial.println("Button was pressed!");
  // Example action: toggle the LED

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    for (int i=0; i<NUMSERVOS; i++){
      pwm.setPWM(servonum[i], 0, pulselen);
    }
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    for (int i=0; i<NUMSERVOS; i++){
      pwm.setPWM(servonum[i], 0, pulselen);
    }
  }
}


void moveEyesAndEyebrows() {
  digitalWrite(LEDPIN, LOW);
  const int steps = 100;  // Number of steps for smoothness

  // Move to the right
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // progress from 0 to 1

    uint16_t eyePos = EYEMIN + t * (EYEMAX - EYEMIN);
    uint16_t browRightPos = 275 + t * (425 - 275);      // Right eyebrow moves up
    uint16_t browLeftPos  = 425 - t * (425 - 275);      // Left eyebrow moves down

    pwm.setPWM(EYESERVO, 0, eyePos);
    pwm.setPWM(EYEBROWRIGHT, 0, browRightPos);
    pwm.setPWM(EYEBROWLEFT, 0, browLeftPos);
    delay(4);
  }

  // Move to the left
  for (int i = steps; i >= 0; i--) {
    float t = (float)i / steps;

    uint16_t eyePos = EYEMIN + t * (EYEMAX - EYEMIN);
    uint16_t browRightPos = 275 + t * (425 - 275);
    uint16_t browLeftPos  = 425 - t * (425 - 275);

    pwm.setPWM(EYESERVO, 0, eyePos);
    pwm.setPWM(EYEBROWRIGHT, 0, browRightPos);
    pwm.setPWM(EYEBROWLEFT, 0, browLeftPos);
    delay(4);
  }

  // Return to the middle
  pwm.setPWM(EYESERVO, 0, EYEMID);
  pwm.setPWM(EYEBROWRIGHT, 0, EYEMID);
  pwm.setPWM(EYEBROWLEFT, 0, EYEMID);
  delay(4);
  digitalWrite(LEDPIN, HIGH);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  // Initialize the button pin as an input
  pinMode(buttonPin, INPUT_PULLUP);
  
  pinMode(LEDPIN, OUTPUT);


  delay(10);

}

void loop() {
  // put your main code here, to run repeatedly:
  // Read the state of the button
  bool buttonState = digitalRead(buttonPin);
  

  if (buttonState == LOW && lastButtonState == HIGH) {
    // Run the custom function
    Serial.println("Button was pressed!");
    //customFunction();
    moveEyesAndEyebrows();
    // Delay a bit to debounce
    delay(50);
    
  }

  // Update the last button state
  lastButtonState = buttonState;

}
