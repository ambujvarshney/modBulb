# Local OTA Update

This application is used to download OTA updates to **modBulb**.

# Usage

* Build the application by using `make` in the gcc directory.

* Build the *modified_bootloader* application.

* Download *modified_bootloader/gcc/exe/application_bootloader.bin* to CC3200 as */sys/mcuimg.bin*.

* Download *local_ota_update/gcc/exe/ota_client.bin* to CC3200 as */sys/mcuimg1.bin*.

* Download the application required to run on **modBulb** to */sys/mcuimg2.bin*. This application should implement a UDP socket that awaits an update request from the update server and reboot to */sys/mcuimg1.bin* (see *local_ota_update/main.c* with `OTA_TEST` compiler flag defined.

* Use the *local_ota_update/server/server.py* python script to initiate and download new images to the board. e.g. `./server.py -ba 192.168.0.100:5001 -sa 0.0.0.0:5001 -um -mf path/to/mcu/binary -uf -ff path/to/fpga/config -vb -us`
