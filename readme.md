# modBulb

Source code of some projects that run on modBulb.

Copyright (C) 2016 Uppsala Networked Objects.

![](https://img.shields.io/badge/license-BSD-green.svg)

# Installation

* Download CC3200 SDK and Service Pack from [TI website](http://www.ti.com/tool/cc3200sdk). The installer is only available for Windows. For Unix users, [Wine](https://www.winehq.org/) can be used to extract the files.

* Clone or download this repository.

* To download to CC3200 board, Install [CC3200 Tool](https://github.com/ALLTERCO/cc3200tool). The tool is built using Python 2.7 and uses standard libraries, so it should support all OSs (We only tested on Ubuntu). Installtion and usage instructions are available at their Github repository.

* Use CC3200 Tool to download the Service Pack binary to the the CC3200 filesystem.

# Usage

* Build the bindaries:
	* GCC users: Place the the downloaded files from this repository within the cc3200-sdk directory following the same directory structure, then use `make` in the gcc directory to build the binaries. If you would rather not place the files within the SDK, modify the Makefile to point to the needed files in the cc3200-sdk directory.
	* CCS users: The projects were not created using CCS, so you need to create the project, link the SDK and add the needed sources.

* Upload to the board using CC3200 Tool.

