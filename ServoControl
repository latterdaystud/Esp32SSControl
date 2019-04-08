#include <PololuMaestro.h>
#include <ArduinoJson.h>

//#include <string>
//#include <iostream>

#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#define raspberrySerial SERIAL_PORT_HARDWARE_OPEN
#else
  //#include <SoftwareSerial.h>
  #include "HardwareSerial.h"
  HardwareSerial maestroSerial(0);
  HardwareSerial raspberrySerial(2);
#endif

/* Next, create a Maestro object using the serial port.

Uncomment one of MicroMaestro or MiniMaestro below depending
on which one you have. */
MicroMaestro maestro(maestroSerial);
//MiniMaestro maestro(maestroSerial);

static void controlServos(void *pvParameters);
static void sendCommands(void *pvParameters);
static void receiveCoordinates(void *pvParameters);

//Interrupt to control firing the laser
static void fire(void);

void moveServos(int x, int y);

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;


SemaphoreHandle_t xBinarySemaphore;

float pulsePerAngle;

typedef struct 
{
  int x;            //Difference in x from center.
  int y;            //Difference in y from center.
  bool flag;        //Target locked
  int centerx;      //Center x location.
  int centery;      // Center y location.
  int totalAnglex;  // Total x angle of the video. 
  int totalAngley;  // Total y angle of the video.
  bool changed;     // Notifies that the data has been updated.
} targetChange;

// A shared memory resource to where the data is passed into from Serial.
targetChange target;

typedef struct
{
  int x;
  int y;
} location;

// Keeps track of the current location of the servos.
location currentLocation;

const byte interruptPin = 25;
const byte interruptPin2 = 33;
const byte interruptPin3 = 32;

 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/*
 * Set up the program for the passing of data. Initialize all the shared variables and values.
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Initialize the test data
  //Initialize all the shared variables and stucts
  currentLocation.x = 4000;
  currentLocation.y = 4000;
  maestro.setTarget(0, 6000);
  //Servo 1 is the y axis.
  maestro.setTarget(1, 6000);
  //Sero 1 is the y axis.
  delay(1000);
  maestro.setTarget(1, 4666);
  delay(1000);
  maestro.setTarget(1, 7333);
  target.x = 0;
  target.y = 0;
  target.centerx = 1;
  target.centery = 1;
  target.flag = false;
  target.totalAnglex = 90;
  target.totalAngley = 90;
  target.changed = false;

  pulsePerAngle = 29.62963;  //For a 270 range servo and 1/4th us range 2000 to 8000
  
  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(26, INPUT);
  digitalWrite(2, LOW);
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  raspberrySerial.begin(9600, SERIAL_8N1,16,17);
  
  maestro.setSpeed(0, 0);
  maestro.setAcceleration(0, 0);
  
  vSemaphoreCreateBinary( xBinarySemaphore );
  
  if (xBinarySemaphore != NULL)
  {
    xTaskCreatePinnedToCore(
      controlServos,
      "Servo Control",
      1000,
      NULL,
      1,
      &Task1,
      0);

      xTaskCreatePinnedToCore(
      receiveCoordinates,
      "Receving Coordinates",
      3000,
      NULL,
      1,
      &Task2,
      1);
      
      //Just for testing that the sending and receiving the coordinates
      xTaskCreatePinnedToCore(
      sendCommands,
      "Test Serial coms",
      1000,
      NULL,
      1,
      &Task3,
      1);

  }
  
  pinMode(interruptPin, INPUT);
  pinMode(interruptPin2, INPUT);
  pinMode(interruptPin3, INPUT);
  digitalPinToInterrupt(interruptPin);
  attachInterrupt(interruptPin, fire, CHANGE);
  digitalPinToInterrupt(interruptPin2);
  attachInterrupt(interruptPin2, fire, CHANGE);
  digitalPinToInterrupt(interruptPin3);
  attachInterrupt(interruptPin3, fire, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Blynk.run();
  //Serial.print("Hello my friend");
}

/*
 * Move the servo the specified amount in the parameters.
 */
void moveServos(int x, int y)
{

  Serial.print(x);
  Serial.print(y);
  int newX = currentLocation.x + x;
  int newY = currentLocation.y + y;
  if(newX < 0)
    newX = 0;
  if(newY < 0)
    newY = 0;
  if(newX > 8000)
    newX = 8000;
  if(newY > 8000)
    newY = 8000;
  //Servo 0 is the x axis.
  //Serial.print("X and Y:");
  //Serial.println(newX);
  //Serial.println(newY);
  maestro.setTarget(0, newX + 2000);
  //Servo 1 is the y axis.
  maestro.setTarget(1, newY + 2000);
  currentLocation.x = newX;
  currentLocation.y = newY;

}

/*
 * Turn the laser on or off based on the intterupt pins.
 */
static void fire(void)
{
    if (digitalRead(interruptPin) && digitalRead(interruptPin2) && digitalRead(interruptPin3))
      digitalWrite(0, HIGH);
    else
      digitalWrite(0, LOW);
    Serial.println("Interrupt occurred");
}

/*
 * This is for testing only.
 * Sends commands to the recieve port to decide what to do.
 */
static void sendCommands(void *pvParameters)
{

  
  int x = 100;
  int y = 90;
  //raspberrySerial.println("Setting up the communication\0");
  vTaskDelay(1000);
  for(int i = 0;; i++)
  {   
    
    DynamicJsonBuffer jBuffer;
      JsonObject& root = jBuffer.createObject();
      root["x"] = 100;
      root["y"] = 90;
      root["h"] = 100;
      root["v"] = 90;
      root["t"] = 100;
      root["p"] = 90;

      root.prettyPrintTo(raspberrySerial);

      raspberrySerial.print('\0');

    //raspberrySerial.print("\n");
    //root.prettyPrintTo(Serial);
      vTaskDelay(150);
      
  }
  
}


/*
 * The highest priority task.
 * Decides how much to turn the servos and controls them.
 */
static void controlServos(void *pvParameters)
{

  for(;;)
  {
    if (target.changed && target.totalAnglex != 0 && target.totalAngley != 0 
        && target.centerx != 0 && target.centery != 0 && target.x != 0 && target.y != 0 && digitalRead(interruptPin3))
    {
      // Make sure that the data doesn't change half way through.
      xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
      float pointsPerAnglex = (target.centerx * 2) / target.totalAnglex;
      float pointsPerAngley = (target.centery * 2) / target.totalAngley;
      float anglex = (target.centerx - target.x) / pointsPerAnglex; 
      float angley = (target.centery - target.y) / pointsPerAngley; 
      int pulsex = anglex * pulsePerAngle;
      int pulsey = angley * pulsePerAngle;
      target.changed = false;
      xSemaphoreGive(xBinarySemaphore); 
      Serial.print("Should have moved");
      moveServos(pulsex, pulsey);
    }
    vTaskDelay(10);
  }
  
}

/*
 * Handles receiving the data.
 */
static void receiveCoordinates(void *pvParameters)
{

  for(;;)
  {
    String content = "";
    char character;
    char json[256];
    int index = 0;
    while(!raspberrySerial.available())
      vTaskDelay(1);
    if(raspberrySerial.available())
    {
      String val = raspberrySerial.readStringUntil('\0');
      Serial.println("Received String");
      Serial.print(val);
      
      DynamicJsonBuffer jsonBuffer(1024);
      JsonObject& root = jsonBuffer.parseObject(val);
      
      if (!root.success()) {   //Check for errors in parsing
        Serial.println("Parsing failed");
      }
      else
      {
        Serial.println("Parsing success");
        //root.prettyPrintTo(Serial);
        // Save all the data passed in through the serial port to a shared struct.
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
        target.x = root["x"];
        target.y = root["y"];
        target.centerx = root["h"];
        target.centery = root["v"];
        target.totalAnglex = root["t"];
        target.totalAngley = root["p"];
        target.changed = true;
        Serial.print(target.x);
        Serial.print(target.y);
        xSemaphoreGive(xBinarySemaphore); 
      }
      
    }
    
    vTaskDelay(1);
    

  }
  
}
