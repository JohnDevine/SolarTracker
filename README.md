# SolarTracker
This is a two axis tracker that uses a pair of BH1750 light sensors 
to track the sun and drive 2 x servos to face the brightest sun.

It first does a full sweep then moves to brightest light. After a time it will sweep a small window and move to that position. If the position is at the very end of the window it will initiate a full sweep.

Microprocessor used: esp32doit-devkit-v1
Light Sensor Used: BH1750 (Board version is GY-302)
Servo Used: MG995 (only capable of 180 sweep)
Development environment : VSC and Platformio
Libraries:
    starmbi/hp_BH1750@^1.0.0
	madhephaestus/ESP32Servo@^0.9.0
	bblanchon/ArduinoTrace@^1.2.0

=======================================================
BH1750 info.
I bought the BH1750s here (No idea why AliExpress gives this huge URL):
https://star.aliexpress.com/share/share.htm?image=U072fee8f59ee43c3995a84eef1e75739R.jpg&businessType=ProductDetail&title=THB%2029.42%20%2016%EF%BC%85%20Off%20%7C%20WAVGAT%20GY-302%20GY-30%20BH1750%20BH1750FVI%20The%20digital%20optical%20intensity%20illumination%20sensor%20BH1750FVI%20of%20module%20for%20arduino%203V-5V&platform=AE&redirectUrl=https%3A%2F%2Fwww.aliexpress.com%2Fitem%2F4000587263111.html%3F%26srcSns%3Dsns_Copy%26tid%3Dwhite_backgroup_101%26mb%3D78kQwy972NDwX2N%26businessType%3DProductDetail%26spreadType%3DsocialShare

Problems & solutions:
1. I tried quite a few bh1750 libraries and found quite a few could not handle multiple sensors active along with the servo driver. This one worked: hp_BH1750 by Stefan Armborst

2. The sensor gives quite a varied reading. I found that the first reading is usually a real outlier so I use the
multipleReadLightSensor routine that discards the first reading and does 

int lightSensorMultipleReadCountNeeded = 5; // Number of readings needed to get a valid reading

reads and after discarding the first reading , averages the others. Also from the datasheet I set the wait time (120ms) to 

int lightSensorWaitReadTime = 120;

after each sense before reading value.

3. The sensor overloads when in bright sun. The sensor can read lux from 1 - 65535 (see datasheet) with an accuracy of 20%. The brightest sunlight is 120,000 lux (wikipedia) so you need a 50% neutral density filter (An ND2 filter) above the sensor to get values in range.
=======================================================
Servo Info:
The servos I used are MG995. They can handle quite a load. I bought them here:
https://star.aliexpress.com/share/share.htm?image=U242cb795d7ad461989c7349980b2b10ai.jpg&businessType=ProductDetail&title=THB%20483.37%20%2012%EF%BC%85%20Off%20%7C%204pcs%2013kg%2015kg%20Metal%20Servos%20Digital%20MG995%20MG996%20MG996R%20Servo%20Metal%20Gear%20motor%20for%20Futaba%20JR%20Car%20RC%20Helicopter%20Boat%20Diy%20toys&platform=AE&redirectUrl=https%3A%2F%2Fwww.aliexpress.com%2Fitem%2F4000536728030.html%3F%26srcSns%3Dsns_Copy%26tid%3Dwhite_backgroup_101%26mb%3D78kQwy972NDwX2N%26businessType%3DProductDetail%26spreadType%3DsocialShare

Problems & solutions:

1. The PWM can be driven directly off the ESP32. The power though for small sized solar panels (40cm diagonal) is too much draw for the ESP32 and causes intermittent problems particularly on big sweeps (10ma at idle, 1200ma stall). Also note that the speed is dependent on the voltage. I used a voltage regulator and set it to 6V. NOTE this is very important, see next problem.

2. There is no feedback from the servo. This means that if you are at 0 degrees and tell it to go to 180 it will take quite a while to get there and there is no way of knowing if it gets there. (I thought of monitoring the amperage usage on the servo and watching it go to a resting amperage .. maybe later). Given that I use a varying delay based on the previous angle and the new angle required. 
The Delay in ms is 

int track60DegreeDelayMs = 300;

needed to force last command thru to the servo.
For the MG995 the spec sheet says it needs .2 sec to do 60 degrees on 4.8 v Notice for different voltages it can be different (higher voltage less delay) I have chosen .3 secs as I found this from testing

NB I have not allowed for slowing due to loading

3. Different servos have different PWM maximum and minimum pulse widths. You need to look at the spec sheet to get them and also it is worth using a testbed to try them out. I have a program here to do that "TestServo".
Values for the Aliexpress MG995

#define DEFAULT_uS_LOW 500
#define DEFAULT_uS_HIGH 2500

4. Like almost all cheap servos they only travel 180 degrees. So to make it track a full 180 you can either mod the servo (lots of stuff on the internet), add a 2:1 gear on the tracker or leave it at 180 and make sure that the morning sun is pointing close to the 0 degree position. The last works for land based systems but alas I have a boat and "the land" is continuously moving.

=======================================================

Other Stuff.
1. The time between checking if the sun is still good is:
The sun's azmith moves approximately 12 degrees per hour at the equator in winter
and 15 degrees per hour at the equator in summer.
The sun's altitude moves approximately 6 degrees per hour at the equator in winter
and 9 degrees per hour at the equator in summer.

So to track the sun in all seasons to 1 degree accuracy we need to take a reading
every 5 minutes .. 5*60 = 300 seconds.

#define SUN_TRACK_READING_INTERVAL_SECONDS 300

On a boat this needs to be a much shorter time.

2. When the tracker wakes up each time it checks either side of where it is positioned to find the strongest sun. It uses a window 

int trackWindowSize = 10;    // Window is this size 
degrees either side of current position.


When it is looking in this window it will move in increments of 

int trackFineStepSize = 1;   // degrees

If it hits the end of the window with a maximum reading it will initiate a full scan to find the sun again. It really only does this on a boat that keeps swinging around.
The full scan  will move in increments of 

int trackCoarseStepSize = 10; // degrees
=======================================================