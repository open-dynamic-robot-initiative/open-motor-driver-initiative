# uOmodri

Software for the Open Motor Driver Initiative project - uOmodri board

Required installs
In order to use the UOMODRI projects, you must install Code Composer Studio and
the TI C2000Ware-MotorControl SDK.
To download Code Composer Studio, use this link:
https://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html
For Linux users, don't forget to install USB drivers (everything is explained
on Texas Instrument website).
When installing CCS, prefer a custom installation and check
"C2000 real-time MCUs" installation.
To dowload the C2000Ware-MotorControl SDK and exctract it, you must use a Windows machine.
However it is possible to execute the installer on Windows and then get all the
files on a USB key for instance.
Here is the dowload link: https://www.ti.com/tool/C2000WARE-MOTORCONTROL-SDK

Repository description
This repository contains three projects:


Omodri_FOC: Main project. Implements current, speed and position control
for PMSM along with a SPI communication for commands or debugging.


How to import these projects
After cloning the repository, you can import the projects by doing Project > Import CCS project.
The last thing you must do before being able to compile the projects is to define
the C2000Ware SDK location in your computer.
To do so, right click on a project, and then Properties > Resource > Linked Resources.
Now modify the MOTORWARE_ROOT path variable to the SDK folder path.
The folder should be named something like "C2000Ware_MotorControl_SDK_3_00_01_00"
for the 3.00.01.00 version.