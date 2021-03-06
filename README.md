# 2017Fall-SmartGarbageCar

## Motivation
We got inspiration of this project from the video of an autonomous trash can on wheels that can position itself underneath pieces of garbage one tosses in its general vicinity. This original work is based on wireless communication with MicroSoft's motion sensing camera and a powerful program that can predict where the garbage will land. In consideration of our limited devices, we proposed this project to design a smart garbage car that can detect and clean up garbages in a small area. And we implemented it on Raspberry Pi and Arduino with technique of BLE and openCV.

## Dependency
- numpy
- openCV
- pexpect
- pygatt

## Implement
The implement of this project is composed of three parts: 
- the detection of garbage and car on Raspberry Pi
- the movement of car
- the communication between car and Raspberry Pi
### Movement
The car is driven by two parallax continuous rotation servos, which are controlled by an Arduino Uno. The servo can be controlled by writing its duty cycle using Arduino `<servo.h>` libriray. Before mounted to the car, the servos should be calibrated by adjusting the  potentiometer in the servo using a screwdriver, while the "stay-still" signal is sending to the servo. After doing that, the servo will rotate forward if its duty cycle is set above the stay-still signal, while it will rotate reversely if the duty cycle is lower than that. 
#### Movement Command
The car can take five command: forward, backward, turn left, turn right and pick the garbage, and we control the moving distance of the car by setting the time interval between the rotation signal and stay-still signal. 

When designing the scheme of controlling the car's movement, we considered 2 different plans. The first one is sending it the angle and distance between the itself and the garbage, but we found it quite difficult to control the car to turn a degree precisely, because the surface of both the wheels and the ground are very smooth and the fictional force is not enough. And sometimes they get stuck when a rotation signal is received. So we decided not to change its direction by the degree of angles and switch to the second plan. In the plan B, we control the car to move only horizontoally or vertically regarding to its former direction. And the only rotation angle is 90 degree, which is much easier to tune in the Arduino code. 

### Detection
### Trash detection
The method we are using for trash detection is based on background subtraction. We use the MOG background subtraction detector provided by OpenCV: cv2.createBackgroundSubtractorMOG in detection. Therefore, the program requires an initial trash-free background during initialization stage. When a trash is detected, the detection is memorized until the system detects that the car move to the trash for pickup.

<div style="text-align:center">
<img src ="images/图片2.png" />
</div>

This is the picture of the detection while the left picture is the raw image seen by the camera and the background subtracted image is on the right. Notice that when a trash stay still for enough of time, it is considered as the background itself. Solving this problem require a initial picture of a clean background to do difference alongside by the subtractor.

![alt text](images/图片3.png)

### Navigation
#### Path
One big problem of moving the car to pick up the garbage is to find an appropriate path. Since we do not have an extra camera on the car, it is impossible for the car to know where to go and we have to calculate the path for the car so that it can get to the right position. However, the direction of car can not be directly detected by the camera and it is changing all the time. Therefore, we record the original direction of the car at the inital stage, and recalculate it when the car moves.

In this project, we first use a method of calculating the running average of the image provided by the camera, and use the image difference to detect the inital location of the car. Assuming that the car is in the view at the first place, such detection is done by calling the car to rotate a full cycle. After that we use the Median Flow tracker provided by OpenCV to track the movement of the car. Such tracker can be replaced by using Machine Learning technique to train a specific classifier for more accurate tracking and better detection in practice.

Also, in order to provide a better navigation path, calibration of the camera is needed. It requires a chessboard-like object which size is known in advance to calibrate and calculate the homography between the camera view and real world. In this project we used a [7x9 chessboard](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf) from the web to calibrate.

### Communication
The Raspberry Pi and Arduino communicate with each other over BLE connection. In this project, the Arduino works as a BLE peripheral and provide GATT service, while the Raspberry Pi works as a central and a client. The transmission of information is implemented writing value to a GATT characteristic.
#### BLE module
We use an HM10 module to make the arduino work as a BLE peripheral. HM10 has 6 pins and we only need 4 of them in this case. The RX and TX pin are connected to Arduino digital pins. The VCC and GND pin are also connected to 5V and GND on the Arduino respectively. 

![alt text][img]

[img]:http://fab.cba.mit.edu/classes/863.15/doc/tutorials/programming/bluetooth/bluetooth2.jpg "HM10 module"

The HM10 module can be configured using its provided set of AT commands. There are some useful AT commands that we used when setting up BLE connection.

- AT+ADDR?: get the address of HM10 module
- AT+BAUD9600: set the baud rate to 9600 (BLE default)
- AT+ROLE0: set its role as BLE peripheral
- AT+UUID?: get the uuid of its provided service
- AT+FFE2: add a second characteristic to its service. 

By default, there is a single custom characteristic under a custom service provided by the HM10 module, and we can add a second write only characteristic using the AT command mentioned above. After configuration, the Arduino can get commands from Raspberry Pi by reading the value of that characteristic.

#### GATT programming
The program of implementing the RPi as a BLE center is based on `pygatt` library, which provides a Pythonic API by wrapping up gatttool command line utility. 

To connect the raspberry pi with HM10 module, we first start the adapter and use it to connect the BLE module by its address.
```
self.address = address
self.type = addr_type
self.write_uuid = "0000ffe2-0000-1000-8000-00805f9b34fb"
try:
	self.adapter = pygatt.GATTToolBackend()
	self.adapter.start()
	print "===== adapter.connect() ====="
	self.device = self.adapter.connect(address, address_type=addr_type)
	print "address: " + str(self.device._address)
except:
  print "Cannot start adapter"
```
As for write characteristic value, the library provides a `char_write` function. All it needs is the uuid of the write-only characteristic.
```
device.char_write(uuid, in_buf[:20])
```
Since it is very likely that the openCV program is not in the same file as the pygatt code, this part is implemented using multi-thread technique as a bridge between two parts of the program. (Although we coded them into the same file in the end, we kept the multi-thread part.)

![alt text](images/img4.png)

To communicate with the main thread, we define a writing event. The main thread will wait for the event and start to process it after the event is set.

Because Arduino communications with the HM10 module over serial connections and it reads information as a sequence of chars, so the commands sent from RPi to Arduino are encoded as a formed sequence of chars, which will be decoded char by char in the Arduino program.

## Reference
- [Home surveillance and motion detection with the Raspberry Pi, Python, OpenCV, and Dropbox](https://www.pyimagesearch.com/2015/06/01/home-surveillance-and-motion-detection-with-the-raspberry-pi-python-and-opencv/)
