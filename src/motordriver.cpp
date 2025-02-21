

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <AccelStepperWithDistance.h>


#define COLUMS           20   //LCD columns
#define ROWS             4    //LCD rows
#define LCD_SPACE_SYMBOL 0x20 //space symbol from LCD ROM, see p.9 of GDM2004D datasheet

// lcd plugged into D1,D2 (slc,sda)

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 D3
#define IN2 D4
#define IN3 D5
#define IN4 D6

int sensorPin = A0;        // select the input pin for the potentiometer
int sensorValue = 0;       // variable to store the value coming from the sensor
const int buttonPin = D7;  // the number of the pushbutton pin

int buttonState = 0;  // variable for reading the pushbutton status

// initialize the stepper library
//AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
AccelStepperWithDistance stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

void setup() {
   Serial.begin(9600);

     pinMode(buttonPin, INPUT);
  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  stepper.setStepsPerRotation(200);   // 1.8° stepper motor
  stepper.setMicroStep(16);           // 1/16 microstepping
  stepper.setDistancePerRotation(8);  // 8mm per rotation
  stepper.setAnglePerRotation(360);   // Standard 360° per rotation
  

  /*
  // Move to 50mm
  stepper.runToNewDistance(50);
  Serial.print("Current position: ");
  Serial.println(stepper.getCurrentPositionDistance());
  
  // Move relatively by -20mm
  stepper.runRelative(-20);
  Serial.print("New position after relative move: ");
  Serial.println(stepper.getCurrentPositionDistance());
  
  // Move to 90° angle
  stepper.runToNewAngle(90);
  Serial.print("Position after moving to 90°: ");
  Serial.println(stepper.getCurrentPositionDistance());
  
  // Set up a move to 100mm (but don't execute it yet)
  stepper.moveToDistance(100);
  */
  /*stepper.runToNewDistance(100);
  Serial.println("move 1");
  stepper.runToNewDistance(200);
  Serial.println("move 2");
  */

 while (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS, 4, 5, 400000, 250) != 1) //colums, rows, characters size, SDA, SCL, I2C speed in Hz, I2C stretch time in usec 
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

    lcd.print(F("PCF8574 is OK...")); //(F()) saves string to flash & keeps dynamic memory free

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("set distance:");
  lcd.setCursor(0, 1);
}

int distance = 0;
int movedDistance = 0;

void loop() {
  // Execute the move set up in setup()
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  sensorValue = analogRead(sensorPin);
  // turn the ledPin on
  Serial.println(sensorValue);

  distance = map(sensorValue, 1, 1024, 0, 200);

  Serial.println(distance);

  lcd.setCursor(0, 1);
  lcd.print("       ");
  lcd.setCursor(0, 1);
  lcd.print(distance);
  lcd.print(" mm");


  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    Serial.println("button high");
    if (stepper.isRunning() == false) {
      movedDistance = movedDistance + distance;
      lcd.setCursor(8, 1);
      lcd.print(movedDistance);
      lcd.print(" mm");
      stepper.runToNewDistance(movedDistance); // try to keep movin
    }
  } else {
    // turn LED off:
    Serial.println("button low");
    // doesnt do anything because the stepper blocks code execution
    /*if (stepper.isRunning() == true) {
      lcd.setCursor(0, 8);
      int distanceLeft = stepper.distanceToGo();
      lcd.print(distanceLeft);
      lcd.print(" left");
    }*/
  }
  delay(100);
}
