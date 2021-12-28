#define ARDUINOTRACE_ENABLE 0 // Enable ArduinoTrace (0 = disable, 1 = enable)
#include <ArduinoTrace.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <hp_BH1750.h>

Servo trackAcrossServo; // Servo for tracking across the sun
Servo trackUpDownServo; // Servo for tracking up/down the sun

// Variable to store servo position
int trackAcrossPos = 0;
int trackUpDownPos = 0;

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int trackAcrossPin = 18;
int trackUpDownPin = 4;

// Delay needed to force last command thru to servo (Deadband)
// For the MG995 the spec sheet says it needs .2 sec to do 60 degrees on 4.8 v
// Notice for different voltages it can be different (higher voltage less delay)
// I have chosen .3 secs as I found this from testing
// NB I have not allowed for slowing due to loading
int track60DegreeDelayMs = 300;

// Values for TowerPro MG995 large servos (and many other hobbyist servos)
//#define DEFAULT_uS_LOW 1000        // 1000us
//#define DEFAULT_uS_HIGH 2000      // 2000us

// Values for TowerPro SG90 small servos & Aliexpress MG995
//#define DEFAULT_uS_LOW 500
//#define DEFAULT_uS_HIGH 2500

int trackMinPulseWidth = 500;
int trackMaxPulseWidth = 2500;

// Step size for track across servo
int trackCoarseStepSize = 10; // degrees
int trackFineStepSize = 1;    // degrees
int trackWindowSize = 5;     // Window is this size degrees either side of current position

// BH1750 Sensors
hp_BH1750 lightSensor1;
hp_BH1750 lightSensor2;

// BH1750 Sensor readings
float lightSensor1Reading = 0;
float lightSensor2Reading = 0;

// BH1750 wait read time (in ms) (Data sheet says 120 max)
int lightSensorWaitReadTime = 120;
int lightSensorMultipleReadCountNeeded = 4; // Number of readings needed to get a valid reading

// BH1750 Sensor status
bool lightSensor1Status = false;
bool lightSensor2Status = false;

bool lightSensorForceRead = true; // Force a reading from the BH1750 sensor

// Create an array of lightSensor readings from 0 to 180 degrees in 1 degree steps
// The array is used to determine for the position of the servo the light intensity
// at that position
// The array is populated by the lightSensor readings
float lightSensorReadingsAcross[181]; // 0 to 180 degrees in 1 degree steps
float lightSensorReadingsUpDown[181];

/* The sun's azmith moves approximately 12 degrees per hour at the equator in winter
 * and 15 degrees per hour at the equator in summer.
 * The sun's altitude moves approximately 6 degrees per hour at the equator in winter
 * and 9 degrees per hour at the equator in summer.
 *
 * So to track the sun in all seasons to 1 degree accuracy we need to take a reading
 * every 5 minutes.
 */
#define SUN_TRACK_READING_INTERVAL_SECONDS 300

// BH1750 Sensor initialization
bool initLightSensor(hp_BH1750 &sensor, int pin)
{
    if (sensor.begin(pin) == false)
    {
        TRACE();

        return false;
    }
    else
    {
        TRACE();
    }

    if (sensor.calibrateTiming() != BH1750_CAL_OK)
    {
        TRACE();

        return false;
    }
    else
    {
        TRACE();
    }

    if (sensor.start(BH1750_QUALITY_HIGH2, BH1750_MTREG_DEFAULT) != true)
    {
        TRACE();

        return false;
    }
    else
    {
        TRACE();
    }

    return true;
}

float singleReadLightSensor(hp_BH1750 &sensor)
{
    float sensorLuxReading = 0;
    sensor.start();
    // delay(lightSensorWaitReadTime);

    while (sensor.hasValue(lightSensorForceRead == false))
    {
        TRACE();
        delay(lightSensorWaitReadTime);
    }
    sensorLuxReading = sensor.getLux();
    TRACE();
    DUMP(sensorLuxReading);
    return sensorLuxReading;
}
// do lightSensorMultipleReadCountNeeded reads of light sensor and average readings
float multipleReadLightSensor(hp_BH1750 &sensor)
{
    float lightSensorReading = 0;
    for (int i = 0; i < lightSensorMultipleReadCountNeeded; i++)
    {

        // First read is always low an outlier so discard it
        if (i == 0)
        {
            lightSensorReading = singleReadLightSensor(sensor);
            lightSensorReading = 0; // discard first reading
        }
        else
        {
            lightSensorReading += singleReadLightSensor(sensor);
        }
    }
    return lightSensorReading / (lightSensorMultipleReadCountNeeded - 1); // average of readings minus the outlier
}

// Test light sensor
float testLightSensor(hp_BH1750 &sensor)
{
    // do a test read and return value

    float testReading = multipleReadLightSensor(sensor);
    TRACE();
    DUMP(testReading);

    return testReading;
}
// Move servo to position
void moveServo(Servo &servo, int position)
{
    int servoDelayNeeded = 0;
    TRACE();
    DUMP(position);

    // Calculate delay needed to let servo move to position
    int oldPositionDegrees = servo.read();
    if (oldPositionDegrees != position)
    {
        if (oldPositionDegrees < position)
        {
            servoDelayNeeded = position - oldPositionDegrees;
        }
        else
        {
            servoDelayNeeded = oldPositionDegrees - position;
        }
        servoDelayNeeded = (servoDelayNeeded * track60DegreeDelayMs) / 60; // Convert to ms
        TRACE();
        DUMP(servoDelayNeeded);

        servo.write(position);

        // Wait for servo to move to position
        delay(servoDelayNeeded); // Delay needed to force last command thru to servo (Deadband)
    }
    TRACE();
}
bool initServo(Servo &servo, int pin)
{
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo.setPeriodHertz(50); // standard 50 hz servo
                              // different servos may require different min/max settings
                              // for an accurate 0 to 180 sweep

    if (servo.attach(pin, trackMinPulseWidth, trackMaxPulseWidth) != 1)
    {
        TRACE();

        return false;
    }
    else
    {
        TRACE();
    }
    moveServo(servo, 90); // Set servo to 90 degrees
    return true;
}

// Test servo
bool testServo(Servo &servo)
{
    TRACE();
    // Loop 10 times and set servo to 0 to 180 degrees
    for (int i = 0; i < 10; i++)
    {
        moveServo(servo, 0);   // Set servo to 0 degrees
        moveServo(servo, 180); // Set servo to 180 degrees
    }

    TRACE();
    for (int i = 180; i >= 0; i -= 10)
    {
        moveServo(servo, i); // move servo to position i
    }
    TRACE();
    for (int i = 0; i <= 180; i += 10)
    {
        moveServo(servo, i); // move servo to position i
    }

    return true;
}
// Move servo from 0 to 180 degrees and take a light reading at each degree.
// The light sensor readings are stored in the array passed in.
// Paramaters are the Array address, the light sensor address, and the servo address
// This always assumes array is of size 181
void sweepServoReadLight(float *lightSensorReadings, hp_BH1750 &lightSensor, Servo &servo, int startDegrees, int endDegrees, int stepDegrees)
{
    TRACE();
    DUMP(startDegrees);
    DUMP(endDegrees);

    // check for valid start and end degrees and if lower than 0 or higher than 180

    if (startDegrees < 0)
    {
        startDegrees = 0;
    }
    if (endDegrees > 180)
    {
        endDegrees = 180;
    }
    if (startDegrees > endDegrees)
    {
        int temp = startDegrees;
        startDegrees = endDegrees;
        endDegrees = temp;
    }

    // set array to 0
    for (int i = 0; i < 181; i++)
    {
        lightSensorReadings[i] = 0;
    }
    // Loop through servo positions from startDegrees to endDegrees
    for (int i = startDegrees; i <= endDegrees; i = i + stepDegrees)
    {
        moveServo(servo, i);                                           // move servo to position i
        lightSensorReadings[i] = multipleReadLightSensor(lightSensor); // Read light sensor
        TRACE();
        DUMP(i);
        DUMP(lightSensorReadings[i]);
    }
}
// Return index to highest value in array
int findMax(float *array, int arraySize)
{
    int maxIndex = 0;

    float maxValue = array[0];
    for (int i = 0; i < arraySize; i++)
    {
        if (array[i] > maxValue)
        {
            maxIndex = i;
            maxValue = array[i];
        }
    }
    return maxIndex; // return index
}
// Sweep across servo from startDegree to endDegree taking light readings to find the best position and position servo there
// Paramaters are the Array address, the light sensor address, the servo address, and the start and end degrees
// return the best position found or -1 if no position found
int sweepServoFindBestLightAndPositionThere(float *lightSensorReadings, hp_BH1750 &lightSensor, Servo &servo, int startDegrees, int endDegrees, int stepDegrees)
{
    TRACE();
    sweepServoReadLight(lightSensorReadings, lightSensor, servo, startDegrees, endDegrees, stepDegrees);
    int maxIndex = findMax(lightSensorReadings, 181);
    moveServo(servo, maxIndex);
    return maxIndex;
}

// Main Setup Routine
void setup()
{
    // Initialize Serial Port
    Serial.begin(115200);

    // Initialize BH1750 Sensors
    if (initLightSensor(lightSensor1, BH1750_TO_VCC) == false)
    {
        Serial.println("Failed to initialize BH1750 Sensor 1");
        lightSensor1Status = false;
    }
    else
    {
        lightSensor1Status = true;
    }

    if (initLightSensor(lightSensor2, BH1750_TO_GROUND) == false)
    {
        Serial.println("Failed to initialize BH1750 Sensor 2");
        lightSensor2Status = false;
    }
    else
    {
        lightSensor2Status = true;
    }

    // Initialize Servos
    if (initServo(trackAcrossServo, trackAcrossPin) == false)
    {
        Serial.println("Failed to initialize trackAcrossServo");
    }

    if (initServo(trackUpDownServo, trackUpDownPin) == false)
    {
        Serial.println("Failed to initialize trackUpDownServo");
    }
}

// Main loop
void loop()
{
    int maxIndex;
    int oldMaxIndex;

    // Test servos
    // testServo(trackAcrossServo);
    // testServo(trackUpDownServo);

    // Test BH1750 Sensors
    /* if (lightSensor1Status)
    {
        lightSensor1Reading = testLightSensor(lightSensor1);
    }
    if (lightSensor2Status)
    {
        lightSensor2Reading = testLightSensor(lightSensor2);
    }
    */

    // Initialise by coarse sweeping across upDownServo and point at strongest reading
    maxIndex = sweepServoFindBestLightAndPositionThere(lightSensorReadingsUpDown, lightSensor2, trackUpDownServo, 0, 180, trackCoarseStepSize);

    oldMaxIndex = maxIndex; // initialise oldMaxIndex to current maxIndex

    while (true)
    {

        // look trackWindowSize degrees either side of current posn for new max and move there
        // If the boat moves a lot it will do a full sweep again.

        Serial.println("maxIndex: ");
        Serial.println(maxIndex);
        Serial.println("oldMaxIndex: ");
        Serial.println(oldMaxIndex);
        Serial.println("trackWindowSize: ");
        Serial.println(trackWindowSize);

        if (maxIndex == oldMaxIndex - trackWindowSize || maxIndex == oldMaxIndex + trackWindowSize)
        {
            maxIndex = sweepServoFindBestLightAndPositionThere(lightSensorReadingsUpDown, lightSensor2, trackUpDownServo, 0, 180, trackCoarseStepSize);
            oldMaxIndex = maxIndex; // update oldMaxIndex to current maxIndex
        }
        else
        {
            oldMaxIndex = maxIndex;
            maxIndex = sweepServoFindBestLightAndPositionThere(lightSensorReadingsUpDown, lightSensor2, trackUpDownServo, maxIndex - trackWindowSize, maxIndex + trackWindowSize, trackFineStepSize);
            // Every SUN_TRACK_READING_INTERVAL_SECONDS
            delay(SUN_TRACK_READING_INTERVAL_SECONDS * 1000);
            // delay(1000);
        }
    }
}
