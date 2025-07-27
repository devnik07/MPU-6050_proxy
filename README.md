# NIC: Natural Interaction in the CAVE environment using an Arduino motion controller
This project uses either an Arduino Nano 33 BLE with it's on-chip LSM9DS1 9-axis IMU (Gyroscope, Accelerometer, Magnetometer) or an Arduino Uno R3 in combination with the GY-521 MPU-6050 module (Gyroscope, Accelerometer) for measuring the orientation (Roll, Pitch and Yaw angles) of the module. 
Moreover, a joystick reader is implemented for transmitting joystick inputs while a Python script is provided to visualize the measured orientation.

## Setup
### Setting up Arduino Uno R3 and GY-521
Wire the Arduino and GY-521 module like this:
![grafik](https://github.com/user-attachments/assets/84fbc1d5-1d65-4000-84ce-6ba82286a07e)

### Setting up the Joystick
Wire the Arduino Nano 33 BLE/ Uno R3 and the joystick like this:
![grafik](https://github.com/user-attachments/assets/8dc76d6c-773c-491d-bbbd-98c09dda112f)

### Setting up PlatformIO
This project uses PlatformIO for compiling, preparing and uploading the sketches located in the `/src/sketches` folder.
Make sure to install the PlatformIO extension for your preferred IDE (I used Visual Studio Code).

### Setting up the python environment
The python packages `vpython`and `pyserial` are used for visualizing the orientation of the LSM9DS1 IMU (Inertial Measurement Unit) or the MPU-6050 IMU respectively.

Use `pip install vpython` and `pip install pyserial` to install the required packages either globally or locally by setting up a virtual environment.

You may have to specify the port to which the sketches were uploaded to according to the PlatformIO output.
To do so, simply specify your port in the first line of the main function located in `visualization/MPUVisualization/main.py` like this:
```python
arduino_serial = serial.Serial('<port>', 115200)
```

## Arduino Uno R3 Usage
I've implemented two different ways to retrieve the orientation angles of the MPU-6050.

### Default mode

Use the PlatformIO environments annoted with `Default` if you want to have the best possible experience. This mode uses the sensor's on-chip digital motion processing (DMP)
which internally optimizes the computation of the sensor's orientation and provides the results using orientation **Quaternions**.

### Custom mode

Alternatively, you can use the PlatformIO environments annoted with `Custom`. This mode was primarily used for sensor fusion testing. 
In this mode, I've implemented my own computations of orientation angles without using the DMP and therefore renounce the internal optimizations of the chip. 
In this mode, you can choose between different computation options by specifying it in the `runCustom.cpp` sketch.
Available computation options are:
* `ComputationOption::ACC_RP` - Only use accelerometer measurements to compute roll/pitch
* `ComputationOption::ACC_RP_LPF` - Use low pass filter to compute roll/pitch using accelerometer
* `ComputationOption::GYRO_RPY` - Only use gyroscope measurements to compute roll/pitch/yaw
* `ComputationOption::COMPL_RPY` - Use sensor fusion to compute roll/pitch/yaw

Don't use the first two options if you want to visualize the orientation since those only compute roll and pitch and will therefore neglect the yaw angle.
The preferred option is `ComputationOption::COMPL_RPY` because this uses both - the gyroscope and accelerometer - measurements for computing roll, pitch and yaw.
However, the yaw angle is computed by only using gyroscope measurements since yawing cannot be detected by the accelerometer.
Integrating the gyroscope measurements over time will lead to the computed yaw angle drifting - also known as *yaw drift*- because there is no additional component
for balancing these long-term errors. Hence, the `Default` mode is suggested for the best experience.

### Calibration
Ahead of visualizing the orientation of the MPU-6050 for the first time you will have to calibrate the IMU. 
In custom mode, use the corresponding calibration PlatformIO environment `calibrateCustom`.
In default mode, the calibration will start automatically on upload of the `runDefault` environment. Alternatively, you can click on the joystick to start a recalibration.
Do **not move** the MPU-6050 during this process.

### Running the orientation computation
Make sure you have calibrated the MPU-6050 in your selected mode before continuing.
Run the corresponding PlatformIO environment in your selected mode (i.e. `runDefault` or `runCustom`).

## Arduino Nano 33 BLE usage
You can choose to use the controller either in a wired or in a wireless environment by running the `runNanoWired` or `runNano` PlatformIO environment respectively.
Note, that the visualization will only work in the wired environment since the wireless mode uses Bluetooth Low Energy instead of serial communication.
Since the LSM9DS1 module doesn't have an on-chip DMP like the MPU-6050, the ![Madgwick Filter](https://github.com/arduino-libraries/MadgwickAHRS) was used for orientation computation.

### Calibration
The fundamental calibration of the LSM9DS1 IMU works exactly the same as the calibration in the `Default` mode of the Uno R3.
However, unlike the LSM9DS1, the MPU-6050 used with the Uno R3 doesn't have a magnetometer.
This magnetometer requires an extensive calibration method which can't be executed solely on the Arduino because it is very memory-intensive.
Hence, an additional PlatformIO environment called `collectMagData` is provided which simply transmits raw magnetometer measurements via serial.
An external magnetometer calibration program like Magneto can then be used to compute the corresponding hard-iron distortions compensation bias and soft-iron distortions compensation matrix. This bias and this matrix have to be copied and pasted into the `B_HI` and `A_SI` fields in the `LSM9DS1Reader.h` file.
A recomputation of these magnetometer calibration parameters will be necessary every time the application environment changes, since this will also change the distortions.


## Visualization
The visualization script receives the orientation in roll, pitch and yaw angles. So, make sure that your corresponding run sketch is computing and printing roll, pitch and yaw instead of quaternions. 
After the corresponding run sketch was successfully uploaded, locate the visualization python script in `visualization/MPUVisualization/main.py` and run this script
(remember to set the correct port). A window showing a model of the Arduino + MPU-6050 setup should be displayed and the physical movements should be visualized.

## Reset calibration
To reset the calibration, use the `resetCalibration` and `resetNano` PlatformIO environments to reset the calibration parameters used for the Uno R3 and Nano 33 BLE respectively. Note, that this will not reset the magnetometer calibration parameters for the Nano 33 BLE.

## Example Usage
I recorded a short video to display an example application of the project:
[![IMAGE ALT TEXT HERE](https://github.com/user-attachments/assets/9202f29c-ae02-4db3-bfd0-5c881df8b29f)](https://youtu.be/KI7Rz95XLvY)
