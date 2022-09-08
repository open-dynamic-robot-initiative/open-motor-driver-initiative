# open-motor-driver-initiative
Open-source drive for brushless motors, following the open dynamic robot initiative 

This contains hardware and firmware for several open source motor control board all based around TMS320F2838x MCU from Texas Instrument. The boards are design to peform sensored FOC control of PMSM ans BMDC motors.
The goal of this is to provide an alternative to close source industrial grade motor controler that does'nt allow custom control design low, custom comunication buses and sensor integration, as well as an alternative to opensource low cost project that make many compromises and limit their performace.

This should only be replicated for specific need, were bandwith, power computation or sensor integration are needed. Often off the shelf solution offer easier and lower cost solution. 

The TMS320F2838x offer a high number of peripheral dedicated to motor control as well as a fair bot of computational power with 3 CPUs and hardware accelerators. It also offers a range of high bandwith comunication buses such as SPI, FSI, EtherCAT, USART, CAN-FD, Ethernet, USB V2, etc. that allow to implement custom network ideal for centralise robotic application. 

The main motivation for board is for it to be integrated in legged robotics to do academic resarch. 

All the content of this repository is under BSD-3 Licence.

## Several motor control boards
Several boards are developed around the same system archinecture to acoomodate different need. (Number of axis, power ranges, from factor etc.)
See below an overview of the baords

[Todo put picture of all the baords, basic specs and link to relevent folder]

### Texas Instrument development boards
This setup used TI dev board and was used for first development, It's not a very practical design
![TI dev boars](doc/dev-board-setup/images/Dual_axis_setup.jpg)
### uOmodri (or udriverv3)
This board was design with to be backward compatible with the udriverv2 used in odri robots.
![uomodri](doc/uomodri/uomodri1.jpg)
### uOmodri signle axis
This board is a project not designed yet. It will be a signe axis version to be attached directly to motors from myactuator.com
### PAL MC 
