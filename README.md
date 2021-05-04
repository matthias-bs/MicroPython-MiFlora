![MiFlora](https://user-images.githubusercontent.com/83612361/117035774-037e9080-ad05-11eb-8cda-cefb2a2a2669.jpg)
![MiFlora_screenshot](https://user-images.githubusercontent.com/83612361/117039563-3591f180-ad09-11eb-9b4b-d4b518d33d1e.png)
# MicroPython-MiFlora

MicroPython library for Xiaomi Mi Flora (aka. flower care) BLE¹ plant sensors

¹ BLE: Bluetooth Low Energy

The library reads the following data from the device:
- firmware version
- battery level
- temperature
- light intensity
- moisture
- conductivity

The sensors' BLE MAC addresses must be supplied by the user
in the variable *miflora_sensors*.

Example:
```
# MAC addresses of two Mi Flora sensors
miflora_sensors = [ 
    bytes(b'\xC4\x7C\x8D\x66\xA5\x3D'),
    bytes(b'\xC4\x7C\x8D\x66\xA4\xD5')
]
```

The MAC addresses can be found as follows:
- Linux: `$ sudo hcitool lescan`
- Android: [nRF Connect for Mobile](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp)
- Windows 10: [Bluetooth LE Explorer](https://www.microsoft.com/en-us/p/bluetooth-le-explorer/9n0ztkf1qd98)

The code was tested with MicroPython V1.15
on ESP32 (build esp32-20210418-v1.15.bin)
with Mi Flora Firmware V3.2.2

MicroPython-MiFlora is based on the following code and documentation:
- https://github.com/ChrisScheffler/miflora/wiki/The-Basics
- https://github.com/basnijholt/miflora/blob/master/miflora/miflora_poller.py
- https://docs.micropython.org/en/latest/library/ubluetooth.html
- https://github.com/micropython/micropython/tree/master/examples/bluetooth
