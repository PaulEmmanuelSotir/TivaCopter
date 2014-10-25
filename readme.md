TivaCopter
========

TivaCopter is a DIY quadcopter project based on tiva connected launchpad using TI RTOS.

Folder structure
--------

* Quadcopter_RTOS : Code composer studio project
* MagMaster : Magnetometer calibration utilities from Yury Matselenak ([DIY drone post](http://diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A1676387))
* Hardware documentation : Documentation about several quadcopter hardware components
* QuadcopterPinMap.pin : Pin map file used with PinMux utility from Texas Intruments to determine launchpad pin map.
* GNSS_Viewer.exe : Executable used to visualize GPS data.

Hardware
--------

* Tiva connected launchpad
* 4x N2830/2212 1000kV ebay brushless motors
* 4x Mystery firedragon 30A Brushless ESCs reflashed with simonk firmware
* 4x GWS 1060 3 blades propellers
* 5000mAh 3s1p zippy flightmax LiPo battery
* HC-05 bluetooth module
* MPU6050 6DOF accelerometer and gyroscope
* HMC5883L 3 axis magnetometer
* 27MHz Radio from remote toy
* F450 quadcopter frame

UART console API
--------

UART console API allow easy command line interface. UART console API provides the following features :
* Simplified UART operations based on modifed UARTStdio util as 'UARTprintf' or 'UARTgets'
* RTOS-compatible
* RTOS-independant
* Multiple UART consoles at the same time
* Automatic help command
* Character deletion
* Command execution aborting with CTRL+C

TivaCopter uses this API to provide command line interface through bluetooth using HC-05 module.
TivaCopter's UART console command exemples:
# set MPU6050 accelerometer range to +- 4g
i2cregrmw 104 28 24 8
# set MPU6050 accelerometer configuration to 24 (0x18)
i2cregw 104 28 24
# read MPU6050 accelerometer configuration
i2cregr 104 28 1
# read MPU6050 raw data (gyroscope, accelerometer and temperature)
i2cregr 104 59 6

I²C Transaction API
--------

I²C Transaction API is an asynchronous Inter Integrated Circuit C Application Programming Interface! I²C Transaction API provides the following features :
* Based on Tivaware I²C driver library
* Asynchronous operations
* RTOS-compatible
* RTOS-independant
* I²C register read, write and read-modify-write operations
* I²C operations dynamic queueing