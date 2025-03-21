# MPU-6050 Orientation visualization
This project uses an Arduino Uno R3 in combination with the GY-521 MPU-6050 module for measuring the orientation (Roll, Pitch and Yaw angles) of the module
using its 3-axis Gyroscope and 3-axis Accelerometer.

Additionally, a Python script is provided to visualize the measured orientation.

## Setup
### Setting up Arduino and GY-521
Wire the Arduino and GY-521 module like this:
![grafik](https://github.com/user-attachments/assets/84fbc1d5-1d65-4000-84ce-6ba82286a07e)

### Setting up PlatformIO
This project uses PlatformIO for compiling, preparing and uploading the sketches located in the `/src/sketches` folder.
Make sure to install the PlatformIO extension for your preferred IDE (i used Visual Studio Code).

### Setting up the python environment
The python packages `vpython`and `pyserial` are used for visualizing the orientation of the MPU-6050 IMU (Inertial Measurement Unit).

Use `pip install vpython` and `pip install pyserial` to install the required packages either globally or locally by setting up a virtual environment.

You may have to specify the port to which the sketches were uploaded to according to the PlatformIO output.
To do so, simply specify your port in the first line of the main function located in `visualization/MPUVisualization/main.py` like this:
```python
arduino_serial = serial.Serial('<port>', 115200)
```

## Usage
I've implemented two different ways to retrieve the orientation angles of the MPU-6050.

### Default mode

Use the PlatformIO environments annoted with `Default` if you want to have the best possible experience. This mode uses the sensor's on-chip digital motion processing (DMP)
which internally optimizes the computation of the sensor's orientation and provides the results using orientation **Quaternions**.

### Custom mode

Alternatively, you can use the PlatformIO environments annoted with `Custom`. In this mode, I've implemented my own computations of orientation angles without using the DMP
and therefore renounce the internal optimizations of the chip. In this mode, you can choose between different computation options by specifying it in the `runCustom.cpp` sketch.
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
To do so, run the corresponding calibration PlatformIO environment in your selected mode (i.e. `calibrateDefault` or `calibrateCustom`).
To run the specified environment use the PlatformIO extension or run `pio run --environment calibrate<Mode>`.
Do **not move** the MPU-6050 during this process.

### Running the orientation computation
Make sure you have calibrated the MPU-6050 in your selected mode before continuing.
After that, run the corresponding run PlatformIO environment in your selected mode (i.e. `runDefault` or `runCustom`).

### Visualization
After the corresponding run sketch was successfully uploaded, locate the visualization python script in `visualization/MPUVisualization/main.py` and run this script
(remember to set the correct port). A window showing a model of the Arduino + MPU-6050 setup should be displayed and the physical movements should be visualized.

