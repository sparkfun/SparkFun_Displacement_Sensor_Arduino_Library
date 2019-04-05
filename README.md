SparkFun Angular Displacement Sensor Arduino Library
===========================================================

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><a href="https://www.sparkfun.com/products/15244"><img src="https://cdn.sparkfun.com//assets/parts/1/3/7/0/3/15244-Bend_Labs_Soft_Flex_Sensor_-_1-Axis-01.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15245"><img src="https://cdn.sparkfun.com//assets/parts/1/3/7/0/4/15245-Bend_Labs_Soft_Flex_Sensor_-_2-Axis-01.jpg"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/15244">SparkFun 1-Axis Digital Flex Sensor (SEN-15244)</a></td>
    <td><a href="https://www.sparkfun.com/products/15245">SparkFun 2-Axis Digital Flex Sensor (SEN-15245)</a></td>
  </tr>
</table>

The single and dual axis digital flex sensors (also known as Angular Displacement Sensors) from Bend Labs have changed the way we think about flex sensors. This flex sensor has 0.1° precision, with a digital I<sup>2</sup>C interface, and an output rate of up to 500Hz, in a single or dual axis configuration. These are truly powerful sensors.

We've written a SparkFun Angular Displacement Sensor Arduino Library to make getting up and running with these sensors a snap. 

This library is based on the Hardware Abstraction Layer written by Bend Labs. If you have a platform that is not supported by this library consider using their [HAL](https://github.com/bendlabs).

The I<sup>2</sup>C address of the sensor is software configurable which means you can hookup over 100 on a single I2C bus!

Thanks to:

* coltonottley - Fixed many bugs and issues around detecting 1-axis sensor properly, proper 1-axis comm, better calibration and polling routines

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Documentation
--------------
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Product Repository](https://github.com/sparkfun/Qwiic_Twist)** - Main repository (including hardware files)

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release any derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
