# ADI EV-HT-200CDAQ1 / VORAGO HT-DAB-1 Read Me

## Overview ##
The EV-HT-200CDAQ1 (from ADI) and HT-DAB-1 (from VORAGO Technologies)  is a complete system reference platform enabling precision data acquisition and control in extreme temperatures up to 200°C. Based upon the AD7981 analog-to-digital SAR converter, this reference design demonstrates a full featured system with two high speed simultaneously sampled channels along with 8 additional multiplexed channels suitable for covering the acquisition requirements for many high temperature, harsh environment applications such as downhole oil and gas instrumentation. The data acquisition front end is connected to the VORAGO Technologies VA10800 ARM® Cortex®-M0 based microcontroller via multiple SPI ports. Once acquired, the data can be processed locally or transmitted via a UART or optional RS485 communications interface. In addition to the hardware, open source firmware uses FreeRTOS with optimized drivers and a data acquisition protocol with multiple modes of operation. Open source host software allows the users to connect to the board via UART to control acquisition modes, view data, log data and perform signal analysis. This platform is suitable as reference design, for rapid prototyping and lab testing of high temperature instrumentation systems. 

This reference design is a collaboration between ADI Alliance members VORAGO Technologies and Petromar Technologies and is the latest addition to the growing ecosystem of products and solutions for high temperature applications.  This board is available for under ADI part EV-HT-200CDAQ1 and VORAGO part HT-DAB-1.


## Project Descriptions ##
/mcu contains the VA10800 ARM® Cortex®-M0 firmware project, utilizing FreeRTOS.  The project was developed with Keil µvision IDE revision 5.23. The code size is below 32k and the free evaluation version can be used. 

/ht_daq_viewer contains the C# project for the HT DAQ Viewer application.  



## User Guide & Downloads ##
A Detailed User Guide for the EVAL-HT-200CDAQ1 / HT-DAB-1, showing how to set up the hardware and use the software,tools, and applications in combination, can be found on the Analog Devices wiki site (https://wiki.analog.com/resources/eval/user-guides/high-temp/ev-ht-200cdaq1), or visit http://www.voragotech.com/products/htdab1.



