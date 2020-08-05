# Autonomous-RC-Car-Milestone-1

One of the my hobbies that played a major role in my interest in cars from a younger age was racing 1/10th scale electric and nitro Remote Controlled Cars competitively. Throughout my high school years, I would travel to different races in Washington, Vancouver, and Victoria with the hopes of getting faster and winning races. However, once university kicked off, all of my cars have been sitting on the shelf collecting dust. 

Until I began browsing YouTube and seeing videos of Donkey Cars and people adding Raspberry Pi's, Nvidia Jetson Nano's, and all sorts of sensors to make the RC cars drive autonomously around a track. A light bulb shined in my head and I knew I wanted to tackle this project. 

I integrated my years of racing remote control car experience with the technical skills gained thus far from my Electrical Engineering degree at UBC to get to the first milestone of this project. Knowing that this is a big task at hand, I am building incrementally and not trying to do everything in one go.

**Project Goal:** 
  * Explore Machine Learning and Deep Learning algorithms to autonomously drive around the track
	    * Start with OpenCV before proceeding with other algorithms
	* Integrate IMU data into the Controls Algorithm 
	* Enhance Python and Embedded programming skills
  * Take it to the local track and autonomously drive around the track

**Milestone 1 Objectives:**
  * Setup all the hardware interfaces using the Nvidia Jetson Nano and the Arduino Nano as the single board computer and microcontroller respectively. 
  * Use computer vision (OpenCV) on the Jetson Nano to identify a green line on the ground (the track) and send out steering and throttle commands to the Arduino Nano, which would     then send out the respective PWM signals to the Servo and the Electronic Speed Controller controlling the Brushed DC Motor.

![Car](https://github.com/oowen98/Autonomous-RC-Car-Milestone-1/blob/master/Photos/IMG-3219.jpg)

Figure 1: Completed car

**Track**
  * 11x7 Feet
![Track](https://github.com/oowen98/Autonomous-RC-Car-Milestone-1/blob/master/Photos/Track.jpg)

Figure 2: Track
  
**High Level Overview - Module Map**
  
![Module Map](https://github.com/oowen98/Autonomous-RC-Car-Milestone-1/blob/master/Photos/System%20Architecture.png)

Figure 3: High Level System Overview
  
**Hardware**

  * Nvidia Jetson Nano
  * Arduino Nano
	* 74LS157 Multiplexer (Or any 2 to 1 Multiplexer)
	* Raspberry Pi Camera V2.1 with wide angle lens attachment

	* Tamiya FF03 (1/10th scale FWD Car)
	* Tamiya Electronic Speed Controller and 540 sized Brushed DC Motor
	  * I also tried Brushless Sensored and Sensorless setups -> More notes on my experiences below
	* Savox SC-1251MG Servo
  * Flysky GT3B Transmitter and Receiver (3 Channels minimum for your TX / RX)
  
![Hardware Schematic](https://github.com/oowen98/Autonomous-RC-Car-Milestone-1/blob/master/Photos/Hardware%20Schematic%20V2.jpg)  

Figure 4: Hardware Schematic

	* Arduino Nano - Nvidia Jetson Connection
		* Connected the GND's together
		* Pin 27 / 28 refer to the i2c bus on the Nvidia Jetson, which correlates to A4 and A5 on the Arduino Nano for the SDA and SCL 

	* Receiver Connection
		* On my Transmitter, I have a 3rd channel which is used to toggle between Transmitter control of the car and autonomous control. This is crucial as I am able to stop the car       if it goes off the path
		* All of the GND's of the receiver are connected to the Arduino Ground and the whole system shares a common ground.
		* Channel 3 Signal Wire is connected to Digital Pin 2 -> This is setup as an interrupt in the Arduino to toggle between high and low for the Multiplexer (More on this in the       software section)
		* Channel 1/2 Signal Wire is connected to B1 and B2 of the multiplexer respectively. (See Figure 2)

	* Arduino Nano - Multiplexer Connection
		* Digital Pin 9 is connected to A1 of the multiplexer (Servo Signal)
		* Digital Pin 6 is connected to A2 of the multiplexer (ESC Signal)
		* Digital Pin 10 is connected to the Select of the multiplexer 
		  * This toggles between transmitter control or Jetson Autonomous Control
		* Vcc is connected to 5V of the Arduino and the Strobe G and GND pins are connected to the Arduino GNDs
		* Note: Any 2 to 1 Data selector / Multiplexer can be used. Or you can use the Pololu 4 Channel RC Servo Multiplexer + PWM Driver instead of my Arduino Nano + Multiplexer        setup.
			* https://www.pololu.com/product/2806
      * Purchasing this from the US and shipping it to Canada was not economically viable so I chose the Arduino Nano + Multiplexer route

![Multiplexer Pin Diagram](https://github.com/oowen98/Autonomous-RC-Car-Milestone-1/blob/master/Photos/Mux%20Pin%20Connection.PNG)

Figure 5: 74LS157 Multiplexer Pin Diagram

	* ESC / Motor Setup
	  * My particular ESC came with two wires, 1 was a regular servo wire for the signal to the controller. The other was just the GND and Power (Voltage of the Battery)
			* The 3 pin servo wire is connected to the output of Y2 of the multiplexer
			* The 2 pin JST Wire is connected to the receiver and the power and GND of the Savox Servo.

		* Currently, I am using a regular Brushed Motor Setup as I do not need a lot of power for my small track. My sensored Motor / ESC setup works; however, the ESC is a bit on         the cheaper end and does not work too well at low speeds, which is why I switched back to the simple DC Motor.
			* Sensored Motor / ESC refers to brushless motors that have hall effect sensors connected to the cap of the motor which sends pulses to the Electronic Speed Controller for         more precise control.
				* Speed Passion 21.5T motor with a Futaba ESC
			* Eventually, I would like to go back to the sensored motor setup and create my own sensor wire where I can collect the pulses of the Hall Effect Sensors to determine how         far the vehicle has travelled. -> I will save this for a later milestone
	
	* Servo 
		* Signal wire of the servo is connected to Y1 Output from the multiplexer
		* Power is connected to the Vbattery coming out of the ESC as mentioned above.
		* Ground is shared by the whole system
		* Note: If your ESC only has 1 servo wire, then just connect the Power of the ESC Wire to the Power of the Servo Wire. It will be 6V (for 2S cars) which is enough to power         the servo.

	* Batteries:
		* 2 cell Lipo Battery (Gens Ace 4200mAH Shorty Back) to power the ESC, Motor, and Servo
		* 10000mAH USB Battery Pack to power the Jetson Nano and Arduino -> Must be able to handle at least 2A; however, I have found that so far, I can run the Jetson in the 10W         configuration and the USB battery pack can handle it so far

**Software**

![Software Architecture](https://github.com/oowen98/Autonomous-RC-Car-Milestone-1/blob/master/Photos/Software%20Schematic.png)

Figure 6: High Level overview of the Software Architecture

	* Main.py
		* Calls all the functions and begins the i2c.py process and the camera.py thread

	* Camera.py
		* Reading from the camera is run in its own thread for faster program performance
		* Using OpenCV, I am then passing the frames from the camera to the Perception.py module

	* Perception.py
		* Using OpenCV, this is where the computer vision is done to detect the green line of the track. 
			1. Convert the frames to HSV Space and determine the lower and upper HSV values of the green line
				* This is done through trial and error and trackbars to determine the optimal values
			2. Using the getPerspectiveTransform, I am transforming the frame from its original view to a birds eye view
			3. Then I am doing pixel summation of the detected green line across the height of the frame. 
				* Take a maximum pixel value and a minimum pixel value (percentage of the maximum pixel value) and determine the indexes where the maximum pixel values exceed the                 minimum pixel value
				* This shows the centre point of the green lane and will be used as the Command for where I want the car to be
			4. The feedback will be the centre of the frame (Width of frame / 2) as the camera is positioned in the centre of the car

	* Controls.py
		* Using the command and feedback from the Perception.py, I am using a PD Controller to determine the Steering Command to send to the Arduino
			* Tuned the P and D values through testing
		* Depending on the steering value, I am outputting a Throttle value based on how much I need to turn
			* No turn = fast
			* Big turn = slow

	* I2c.py
		* Using i2c communication, I am sending the Steering and Throttle commands to the Arduino nano

	* Jetson_Arduino_i2c.ino
		* After receiving the Steering and Throttle Data, I am using the Servo.h library to output the PWM Values to the ESC and Servo
		* As well, I am using Channel 3 on my transmitter to determine if I want to manually control the car or control it autonomously from the Jetson Nano
			* The Ch3 button on my receiver sends out either a long or short PWM pulse and I am using interrupts on the Arduino to determine the PWM signal. This will determine the           output of the multiplexer through the Select pin. 

**Final Result**

https://www.youtube.com/watch?v=YcdA1ICcbnU&feature=youtu.be

Figure 7: Video of autonomous driving. 

The car can continuously drive around the track but due to video size limitations, only 1 lap is shown. Toggle the Channel 3 button to choose between autonomous or manual control.

**Conclusion**

I am extremely pleased with the hardware platform that is currently setup and I believe there are a lot of improvements that can be done to drastically enhance the vehicle and  be able to drive around an actual Remote Control Car Race track. I have learned a lot so far and will continue to progress on this project. 

**Next Steps**
  * Implement Artificial Intelligence / Machine Learning Algorithms that can be used on this vehicle and take advantage of the GPU on the Jetson Nano.
  * Collect IMU Data to improve the Controls and path planning algorithm
  
  





