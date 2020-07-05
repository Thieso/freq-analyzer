# freq-analyzer: An Acceleration Frequency Analyzer

This setup uses a MPU6050 as acceleration sensor which sends data to an Arduino
nano. The arduino forwards the data over USB to a laptop. On the laptop a
frequency analysis is done using the FFT function and the FFT is plotted in real
time. 

## Dependencies

### Arduino Interfacing

Interfacing with the Arduino is done with the
[platformio](https://platformio.org/) library. The included Makefile can be used
to upload the code to the Arduino Nano by running 

```
make upload
```

### Python Modules

For the post processing the following important python modules are used:

| Python Module                         |
| -----------                           |
| [SciPy](https://www.scipy.org/)       |
| [Numpy](https://numpy.org/)           |
| [Matplotlib](https://matplotlib.org/) |

## Electrical Wiring

### MPU6050 Sensor to Arduino Nano

| MPU6050 Pin | Arduino Pin |
| ----------- | ----------- |
| 5V          | 5V          |
| GND         | GND         |
| SCL         | SCL         |
| INT         | INT0        |

### Arduino Nano to PC

The Arduino Nano is connected to the PC by a USB cable.
