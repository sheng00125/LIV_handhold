# LIV_handhold
## 1. Introduction
This repository provides the CAD files of our handheld device, all of the modules are capable of [*FDM (Fused Deposition Modeling)*](https://en.wikipedia.org/wiki/Fused_filament_fabrication) printable. 

We release the all of our schematic files with STL files format (under the “release” directory), which can be imported and printed directly. Moreover, we also release the CAD source files (with suffix “\*.SLDPRT and \*.SLDASM”), which can be opened and edited with [*Solidworks*](https://www.solidworks.com).

**Contributors**: [Sheng Hong](https://github.com/sheng00125) and [Chunran Zheng](https://github.com/xuankuzcr)

<div align="center">
<img src="./pics/cover.jpg"  width="100.0%" />
</div>


## 2. Guide to installation
### 2.1 Assembly instruction
The assembly instructions are shown in the following figure, where we mark each module correspondingly with the name of each STL files.

<p align="center">
  <img src="./pics/1.gif" alt="Image Description" width="48%">
  <img src="./pics/2.gif" alt="GIF Description" width="48%">
</p>


### 2.2 Electronic connection
The guide of electronic connection is shown as follow:
| Livox Avia LiDAR M12 | Peripheral Function     |
|--------|------|
|PIN 1   |Power: +  |
|PIN 2   |Power: -  |
|PIN 7   |Ethernet: right PIN3 |
|PIN 6   |Ethernet: left PIN3 |
|PIN 5   |Ethernet: green PIN2 |
|PIN 4   |Ethernet: green / write|
|PIN 11  |MCU PWM - GND |
|PIN 12  |MCU PWM + PB5 |

### 2.3 Root directory

    ├── My_handhold/ - CAD source files
    │   ├── ...
    ├── livox_ros_driver/ - livox lidar ROS driver
    │   ├── ...
    ├── livox_sdk/ - livox lidar SDK
    │   ├── ...
    ├── mvs_ros_pkg/ - Camera driver
    │   ├── ...
    └── stm32_timersync-open/ - Testing folder
    │   ├── USER/ - Main functionality tests
    │   ├── ...
    └── README.md - Project homepage document
    └── ...

## 3. Open and edit the source files
If you have installed [*Solidworks*](https://www.solidworks.com) on your computer, we strongly recommand you to see more details by openning the assembly file ("**LIVO.SLDASM**").

## 4. Material lists (only for reference)
| Item  | Pics  | Purchasing list <br> (available is <br>  not guaranteed)  |
| :------------: | :------------: | :------------: |
| Livox Avia LiDAR  | <img src="./pics/livox_avia.png" width=40%  />  | [Livox tech](https://www.livoxtech.com/); [DJI store](https://store.dji.com/hk-en/product/livox-avia) |

## 5. License
The source code is released under [GPLv2](http://www.gnu.org/licenses/) license. 

If you use any code of this repo in your academic research, it will be **very very appreciated** if you can cite any of our following papers:
