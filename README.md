![MiFlora](https://user-images.githubusercontent.com/83612361/117035774-037e9080-ad05-11eb-8cda-cefb2a2a2669.jpg)
# MicroPython-MiFlora
MicroPython library for Xiaomi Mi Flora (aka. flower care) BLE plant sensors

The library reads the following data from the device:
- firmware version
- battery level
- temperature
- light intensity
- moisture
- conductivity

The sensors' BLE MAC addresses must be supplied by the user
in the variable *miflora_sensors*.

The MAC addresses can be found as follows:
- Linux: $ sudo hcitool lescan
- Android App "nRF Connect"

The code was tested with MicroPython V1.15
on ESP32 (build esp32-20210418-v1.15.bin)
with Mi Flora Firmware V3.2.2

MicroPython-MiFlora is based on the following code and documentation:
- https://github.com/ChrisScheffler/miflora/wiki/The-Basics
- https://github.com/basnijholt/miflora/blob/master/miflora/miflora_poller.py
- https://docs.micropython.org/en/latest/library/ubluetooth.html
- https://github.com/micropython/micropython/tree/master/examples/bluetooth
