# Software for the Open Motor Driver Initiative project - uOmodri board

## Required installs
In order to use the UOMODRI projects, you must install [Code Composer Studio (CCS)](https://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html) and [C2000Ware library](https://www.ti.com/tool/C2000WARE#downloads) from Texas Instruments. C2000Ware Motor Control SDK is not necessary anymore. 
The [installation](https://software-dl.ti.com/ccs/esd/documents/users_guide/index_installation.html) procedure is available on TI website.
For Linux users, don't forget to install USB drivers (everything is explained on Texas Instrument website).
When installing CCS, prefer a custom installation and check "C2000 real-time MCUs" installation.


## Repository description
This repository contains one project:
	**uOmodri_c28** : Main project. Implements current, speed and position control for PMSM along with a SPI communication for commands or debug. 
3 Files are mainly used for configuration:
- inc\uomodri_user_defines.h : Saves all the defines used in the project.
- inc\uomodri_hal_config_handlers.h : Structures necessaries during initialization phase.
- inc\uomodri_prj_config_handlers.h : Structures used in the running phase of the project.


## How to import these project
After cloning the repository, you can import the projects by doing Project > Import CCS project.
The last thing you must do before being able to compile the projects is to define the path to your C2000Ware library.
To do so, right click on the  project, and then Properties > Resource > Linked Resources.
Now modify the C2000WARE_ROOT path to match your own C2000Ware library folder path.
The folder should be named something like "C2000Ware_4_01_00_00" for the 4.01.00.00 version.
Verify that PARENT_LOC and WORKSPACE_LOC point on your own working workspace.