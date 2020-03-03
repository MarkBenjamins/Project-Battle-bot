#include <ArduinoRobot.h>
#include <Arduino_LCD.h>
#include <Compass.h>
#include <EasyTransfer2.h>
#include <EEPROM_I2C.h>
#include <Fat16.h>
#include <Fat16Config.h>
#include <Fat16mainpage.h>
#include <Fat16util.h>
#include <FatStructs.h>
#include <Multiplexer.h>
#include <SdCard.h>
#include <SdInfo.h>
#include <Squawk.h>
#include <SquawkSD.h>

/**
 * Alle library's die we toevoegen
 */
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <BattleBotDrive.h>

/**
 * Even wat informatie voor coderen
 * 
 * battleBotDrive.drive(0, 0);
 * deze regel hierboven is om de wielen te laten draaien, linker getal is linker wiel en rechter getal is rechter wiel.
 * Bij 0 staan de wielen stil. Bij bijvoorbeeld 50 gaat het wiel met een bepaalde snelheid vooruit en bij -50 gaat die achteruit.
 * 
 * 
 * Hieronder uitleg over LCD Scherm
 * updateLCDText("Text");
 * De regel hierboven is om tekst te laten verschijnen op het bovenste gedeelte op het LCD Scherm.
 * 
 * updateSecondLCDText("Text");
 * De regel hierboven is om tekst te laten verschijnen op het onderste gedeelte op het LCD Scherm.
 * 
 * Let wel op!! Er zijn maar 2 lijnen op het LCD Scherm om tekst neer te zetten. Er kunnen maximaal maar 16 tekens op een regel.
 */

/**
 * Define alle pins die we hebben
 */
#define leftMotorForwardPin 3   // Output pin that is connected to the left motor for forward movement.
#define leftMotorBackwardPin 2  // Output pin that is connected to the left motor for backward movement.
#define rightMotorForwardPin 9  // Output pin that is connected to the right motor for forward movement.
#define rightMotorBackwardPin 4 // Output pin that is connected to the right motor for backward movement.

#define leftInfraredSensor 10   // Input pin that is connected to the left infrared sensor.
#define rightInfraredSensor 11  // Input pin that is connected to the right infrared sensor.

#define ultraEchoPin 12         // Input pin that is connected to the ultra echo sensor.
#define ultraEchoTriggerPin 13  // Input pin that is connected to the trigger from the ultra echo sensor.

#define bluetoothReceivePin A0  // The input pin for receiving bluetooth messages.
#define bluetoothTransmitPin A1 // The output pin for transmitting bluetooth messages.

#define lcdDisplayAddress 0x27  // The address of the LCD display
#define lcdDisplayColumns 16    // The amount of characters each row of the display has.
#define lcdDisplayRows 2        // The amount of rows the display has

#define drivingLowerLimit 45    // The under limit for the motor speed.
#define drivingUpperLimit 255   // The upper limit for the motor speed.

#define defaultDrivingSpeed 10  // The default speed of the cart.

enum
{
    TRUE = 1,
    FALSE = 0
}; // Simpele nummering voor de status van de boolean.

enum TapeDetected
{
    NON_SENSOR,   // Sensor status wanneer beide infrarood sensors geen tape detecteren.
    LEFT_SENSOR,  // Sensor status wanneer de linker infrarood sensor tape detecteert.
    RIGHT_SENSOR, // Sensor status wanneer de rechter infrarood sensor tape detecteert.
    BOTH_SENSOR   // Sensor status wanneer beide infrarood sensors tape detecteren.
};

/**
 * Declaration of variables that store the commands given to the battle bot.
 */
int incommingbluetoothCommand = 0; // Variabele dat het laatste bluetooth bericht stored.
int commandInt = 0;
String commandString = "";
String commandArgument = "";

/**
 * Declaration of variables that store the messages that will get displayed on the LCD screen.
 */
String lcdDisplayText = "";       
String secondLcdDisplayText = ""; 
String debugMessage = ""; 

/**
 * Declaration of variables that are needed to initate objects.
 */
int maxPingDistance = 200;                 
unsigned long previousMillisSendVelocity;  
unsigned long sendVelocityInterval = 3000;

/**
 * Variabelen voor autonoom rijden, zijn we nog niet nodig
 */

long startTimer = 0;
boolean endTimer = false;
boolean start = false;
boolean firstTime = true;

/**
 * The initiation of of objects that are used to communicate with the battle bot's modules.
 */
// Create an new serial communication object for bluetooth communication.
SoftwareSerial BTSerial(bluetoothReceivePin, bluetoothTransmitPin);

// Create an new lcd object for displaying debugging messages on the battle bot.
LiquidCrystal_I2C lcd(lcdDisplayAddress, lcdDisplayColumns, lcdDisplayRows);

// Create an new Ping object for measuring the distance to obstacles.
NewPing sonar(ultraEchoTriggerPin, ultraEchoPin, maxPingDistance);

// Create an new MPU6050 object for using the gyroscope and accelerometer.
MPU6050 accelgyro;

// Create an new BattleBot object for controlling the battle bot.
BattleBotDrive battleBotDrive(leftMotorForwardPin, rightMotorForwardPin, leftMotorBackwardPin, rightMotorBackwardPin);

/**
 * Functie om de battleBot te initialiseren
 */ 
void setup() 
{
    // Initialize the I/O pins
    pinMode(leftMotorBackwardPin, OUTPUT);
    pinMode(leftMotorForwardPin, OUTPUT);
    pinMode(rightMotorBackwardPin, OUTPUT);
    pinMode(rightMotorForwardPin, OUTPUT);

    pinMode(leftInfraredSensor, INPUT);
    pinMode(rightInfraredSensor, INPUT);

    pinMode(ultraEchoTriggerPin, INPUT);
    pinMode(ultraEchoPin, INPUT);

    // Start the LCD screen.
    lcd.begin();
    lcd.backlight();
    updateLCDText("Yellow Cyclops 3");
    updateSecondLCDText("Ready to start");
    delay(1000);
    Serial.begin(9600);
    while (!Serial)
    {
    }

    BTSerial.begin(38400);
    while (!BTSerial)
    {
    }
    updateSecondLCDText("Awaiting input..");
}

/**
 * Functie voor het detecteren van tape.
 */
TapeDetected detectTape()
{
    if (digitalRead(rightInfraredSensor) == HIGH && digitalRead(leftInfraredSensor) == HIGH)
    {
        // Both infrared sensors detect tape.
        return BOTH_SENSOR;
    }
    else if (digitalRead(rightInfraredSensor) == HIGH)
    {
        // The left infrared sensor detected tape.
        return LEFT_SENSOR;
    }
    else if (digitalRead(leftInfraredSensor) == HIGH)
    {
        // The right infrared sensor detected tape.
        return RIGHT_SENSOR;
    }
    else
    {
        // Both infrared sensors detected nothing.
        return NON_SENSOR;
    }
}

/**
 * Hieronder staat het begin van het definiëren van het LCD scherm
 */
 
/**
 * Verwijderen van alle tekst op lijn 0 en 1 van het LCD scherm
 */
void clearLcdLine(int lineIndex)
{
    lcd.setCursor(0, lineIndex);
    for (int i = 0; i < 16; ++i)
    {
        lcd.write(' ');
    }
    lcd.setCursor(0, lineIndex);
}

/**
 * Update eerste LCD lijn
 */
void updateLCDText(String ScreenText)
{
    if (lcdDisplayText != ScreenText)
    {
        clearLcdLine(0);
        lcd.setCursor(0, 0);
        lcd.print(ScreenText);
        lcdDisplayText = ScreenText;
        Serial.println(ScreenText);
    }
}

/**
 * Update tweede LCD lijn
 */
void updateSecondLCDText(String secondScreenText)
{
    if (secondLcdDisplayText != secondScreenText)
    {
        clearLcdLine(1);
        lcd.setCursor(0, 1);
        lcd.print(secondScreenText);
        secondLcdDisplayText = secondScreenText;
        Serial.println(secondScreenText);
    }
}

/**
 * Dit is het einde van het definiëren van het LCD scherm.
 */


/**
 * Lijn volgen programma, werkt niet maar is alvast klein begin
 */

/* volg de lijn spel */ /* Controler X */
void followLineProgram()
{
    TapeDetected onSensor = detectTape();
    switch (onSensor)
    {
    case LEFT_SENSOR:
        updateSecondLCDText("Tape right");
        //battleBotDrive.drive(40, -10); werkt
        //battleBotDrive.drive(80, -40);

        battleBotDrive.drive(40, -10);
        battleBotDrive.drive(80, -40);
        break;

    case RIGHT_SENSOR:
        updateSecondLCDText("Tape left");
        //battleBotDrive.drive(-10, 40); werkt
        //battleBotDrive.drive(-40, 80);

        battleBotDrive.drive(-10, 40);
        battleBotDrive.drive(-40, 80);
        break;

    case BOTH_SENSOR:
        updateSecondLCDText("Tape both");
        battleBotDrive.drive(-5, -5);
        break;

    case NON_SENSOR:
        updateSecondLCDText("No tape");
        battleBotDrive.drive(45, 45); // zet terug naar 10
        break;

    default:
        break;
    }
}

/**
 * doolhof programma rechts
 */
void doolhofRechts()
{
    TapeDetected onSensor = detectTape();
    switch (onSensor)
    {
    case RIGHT_SENSOR:
        updateSecondLCDText("Tape left");
        battleBotDrive.drive(-15, 20);
        break;

    case LEFT_SENSOR:
        updateSecondLCDText("Tape right");
        battleBotDrive.drive(20, -15);
        break;

    case BOTH_SENSOR:
        updateSecondLCDText("Tape both");
        battleBotDrive.drive(20, -15);
        break;

    case NON_SENSOR:
        updateSecondLCDText("No tape detected");
        battleBotDrive.drive(50, 50);
        break;

    default:
        break;
    }
}

/**
 * doolhof programma links
 */
void doolhofLinks()
{
    TapeDetected onSensor = detectTape();
    switch (onSensor)
    {
    case RIGHT_SENSOR:
        updateSecondLCDText("Tape left");
        battleBotDrive.drive(-15, 10);        
        break;

    case LEFT_SENSOR:
        updateSecondLCDText("Tape right");
        battleBotDrive.drive(10, -15);
        break;

    case BOTH_SENSOR:
        updateSecondLCDText("Tape both");
        battleBotDrive.drive(-15, 10);
        break;

    case NON_SENSOR:
        updateSecondLCDText("No tape detected");
        battleBotDrive.drive(42, 50);
        break;

    default:
        break;
    }
}

/**
 * parcour
 */
void parcour()
{
    int distance = sonar.ping_cm();
    String distanceLCDText = String(distance) + "cm";
    updateSecondLCDText(distanceLCDText);
    delay(500);
}
/**
 * Begint door de loop() functie, checkt of er bluetooth commands binnenkomen.
 */ 
void receiveAndStoreCommand()
{
    while (BTSerial.available())
    {
        char incoming = (BTSerial.read());

        String data = String(incoming);
        Serial.println(data);
        switch (incoming)
        {
        case 'F':
            commandString = "F";
            break;
        case 'B':
            commandString = "B";
            break;
        case 'L':
            commandString = "L";
            break;
        case 'R':
            commandString = "R";
            break;
        case 'S':
            commandString = "S";
            break;
        case 'w':
            commandString = "F";
            break;
        case 's':
            commandString = "B";
            break;
        case 'a':
            commandString = "L";
            break;
        case 'd':
            commandString = "R";
            break;
        case '1':
            commandString = "1";
            break;
        case '2':
            commandString = "2";
            break;
        case '3':
            commandString = "3";
            break;
        case '5':
            commandString = "5";
            break;
        case 'y':
            commandString = "5";
            break;
        case 'u':
            commandString = "5";
            break;
        case 'i':
            commandString = "5";
            break;
        case 'o':
            commandString = "5";
            break;
        default:
            break;
        }
    }
}

/**
 * Begint door de loop() functie, voert het bluetooth command uit.
 */ 
void executeStoredCommand()
{
    if (commandString == "F")
    {
        updateLCDText("Driving forward");
        updateSecondLCDText("No Game Selected");
        battleBotDrive.drive(100, 200);
    }
    else if (commandString == "B")
    {
        updateLCDText("Driving backward");
        updateSecondLCDText("No Game Selected");
        battleBotDrive.drive(-150, -200);
    }
    else if (commandString == "L")
    {
        updateLCDText("Driving left");
        updateSecondLCDText("No Game Selected");
        battleBotDrive.drive(0, 100);
    }
    else if (commandString == "R")
    {
        updateLCDText("Driving right");
        updateSecondLCDText("No Game Selected");
        battleBotDrive.drive(100, 0);
    }
    else if (commandString == "S")
    {
        still();
    }
    else if (commandString == "1")
    {
        updateLCDText("lijn race");
        followLineProgram();
    }
     else if (commandString == "2")
    {
        updateLCDText("Doolhof Rechts");
        doolhofRechts();
    }
    else if (commandString == "3")
    {
        updateLCDText("Doolhof Links");
        doolhofLinks();
    }
    else if (commandString == "4")
    {
        updateLCDText("Parcour");
        parcour();
    }
    else
    {        
        updateLCDText("Yellow Cyclops 3");
        updateSecondLCDText("Awaiting input..");
    }
}

void still()
{
    //variabelen voor het 'autonoom' rijden
    startTimer = 0;

    endTimer = false;
    start = false;
    firstTime = true;

    //vars for the other games to reset if called.
    updateLCDText("STOP");
    updateSecondLCDText("STOP");
    battleBotDrive.drive(0, 0);
}

/**
 * Dit is de main functie, dit luistert constant naar inkomende commands
 */
void loop() 
{
    receiveAndStoreCommand();
    executeStoredCommand();
}
