###############################################################################
# miflora.py
# 
# MicroPython library for Xiaomi Mi Flora (aka. flower care) BLE plant sensors
# Copyright (C) 04/2021 Matthias Prinke
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# The program reads the following data from the device:
# - firmware version
# - battery level
# - temperature
# - light intensity
# - moisture
# - conductivity
#
# The sensors' BLE MAC addresses must be supplied by the user
# in <miflora_sensors> (see below).
#
# The code was tested with MicroPython V1.15
# on ESP32 (build esp32-20210418-v1.15.bin)
# with Mi Flora Firmware V3.2.2 
#
# Please refer to https://github.com/matthias-bs/MicroPython-MiFlora
# for updates/bug fixes.
#
# Many thanks to everyone who contributed to reverse-engineering the
# Mi Flora communication protocol.
#
# References:
# - https://github.com/ChrisScheffler/miflora/wiki/The-Basics
# - https://github.com/basnijholt/miflora/blob/master/miflora/miflora_poller.py
# - https://docs.micropython.org/en/latest/library/ubluetooth.html
# - https://github.com/micropython/micropython/tree/master/examples/bluetooth
# - https://www.bluetooth.com/specifications/assigned-numbers/
#   (16-bit UUID Numbers Document)
#
# Although the code was intended for integration of Mi Flora devices into
# MicroPython, it might be useful for other BLE GATT devices.
#
# History:
#
# 20210413 Basic functionality implemented
# 20210423 Added evaluation of peripheral name
#          added waiting with timeouts
# 20210504 Implemented AUTO_MODE
#          fixed several bugs and cleaned up code
#          initial release on GitHub
#
# ToDo:
# 
# - 
#
###############################################################################


import ubluetooth
import time
import struct
import ble_advertising
from ble_advertising import decode_services, decode_name
import binascii

from micropython import const

VERBOSITY = 0

# Bluetooth MAC addresses of Miflora sensors
# - Linux: $ sudo hcitool lescan
# - Android App "nRF Connect"
#
# (Example:)
miflora_sensors = [ 
    bytes(b'\xC4\x7C\x8D\x66\xA5\x3D')
#    bytes(b'\xC4\x7C\x8D\x66\xA4\xD5'), # #4
#    bytes(b'\xC4\x7C\x8D\x66\xA1\xEA'), # #3
#    bytes(b'\xC4\x7C\x8D\x66\xA4\xF5'), # #2
#    bytes(b'\x80\xEA\xCA\x88\xFE\xED')  # #1
]

# If SCAN_DEVICES == True, the demo functions start with scanning for devices and the device name and
# the RSSI are retrieved from the scan results. Otherwise the demo functions start with connecting to
# the devices. 
SCAN_DEVICES = True

# The discovery methods are not required for Miflora, because the Services / Characteristics / Value Handles
# are known in advance (see 'References' in the file header), but might be useful in other BLE applications. 
# If _DISCOVER_SERVICES == True, BLE.gattc_discover_services() is called in _IRQ_PERIPHERAL_CONNECT state,
# if  DISCOVER_SERVICES == True, MiFlora.discover_services() is called from the application.
_DISCOVER_SERVICES = False
DISCOVER_SERVICES  = False
# If _DISCOVER_CHARACTERISTICS == True, BLE.gattc_discover_characteristics() is called in _IRQ_GATTC_SERVICE_DONE state,
# if  DISCOVER_CHARACTERISTICS == True, MiFlora.discover_characteristics() is called from the application
_DISCOVER_CHARACTERISTICS = False
DISCOVER_CHARACTERISTICS  = False


# If AUTO_MODE is enabled, the BLE state machine is started with
# MiFlora.scan() or MiFlora.gap_connect() - as needed - and automatically progresses
# through the following stages (AUTO_MODE transitions marked with '*'):
#  1. _IRQ_SCAN_RESULT
#     if completed: -> _IRQ_SCAN_DONE
#  2. _IRQ_SCAN_DONE
#     -> _IRQ_PERIPHERAL_CONNECT
#  3. _IRQ_PERIPHERAL_CONNECT
#     if _DISCOVER_SERVICES: -> _IRQ_GATTC_SERVICE_RESULT
#     else _S_READ_FIRMWARE_DONE
#  4. _IRQ_GATTC_SERVICE_RESULT
#     if completed: -> _IRQ_GATTC_SERVICE_DONE
#  5. _IRQ_GATTC_SERVICE_DONE
#     if _DISCOVER_CHARACTERISTICS: -> _IRQ_GATTC_CHARACTERISTIC_RESULT
#  6. _IRQ_GATTC_CHARACTERISTIC_RESULT
#     if completed: -> _IRQ_GATTC_CHARACTERISTIC_DONE
#  7. _IRQ_GATTC_CHARACTERISTIC_DONE
#     -> _S_READ_FIRMWARE_DONE
#  8. _S_READ_FIRMWARE_DONE
#     -> _S_MODE_CHANGE_DONE
#  9. _S_MODE_CHANGE_DONE
#     -> _S_READ_SENSOR_DONE
# 10. _S_READ_SENSOR_DONE
#
# The application can start the state machine, perform other tasks, eventually wait until
# the state _S_READ_SENSOR_DONE is reached (or a timeout occurred) and finally disconnect
# from the peripheral (MiFlora.disconnect()).
AUTO_MODE = 1

# Time constants (T_WAIT: ms / others: s)
_T_WAIT  = const(100)
_T_RETRY = const(10)
_T_CYCLE = const(20)


# Interrupt request IDs (cf. https://docs.micropython.org/en/latest/library/ubluetooth.html)
_IRQ_CENTRAL_CONNECT                = const(1)
_IRQ_CENTRAL_DISCONNECT             = const(2)
_IRQ_GATTS_WRITE                    = const(3)
_IRQ_GATTS_READ_REQUEST             = const(4)
_IRQ_SCAN_RESULT                    = const(5)
_IRQ_SCAN_DONE                      = const(6)
_IRQ_PERIPHERAL_CONNECT             = const(7)
_IRQ_PERIPHERAL_DISCONNECT          = const(8)
_IRQ_GATTC_SERVICE_RESULT           = const(9)
_IRQ_GATTC_SERVICE_DONE             = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT    = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE      = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT        = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE          = const(14)
_IRQ_GATTC_READ_RESULT              = const(15)
_IRQ_GATTC_READ_DONE                = const(16)
_IRQ_GATTC_WRITE_DONE               = const(17)
_IRQ_GATTC_NOTIFY                   = const(18)
_IRQ_GATTC_INDICATE                 = const(19)
_IRQ_GATTS_INDICATE_DONE            = const(20)
_IRQ_MTU_EXCHANGED                  = const(21)
_IRQ_L2CAP_ACCEPT                   = const(22)
_IRQ_L2CAP_CONNECT                  = const(23)
_IRQ_L2CAP_DISCONNECT               = const(24)
_IRQ_L2CAP_RECV                     = const(25)
_IRQ_L2CAP_SEND_READY               = const(26)
_IRQ_CONNECTION_UPDATE              = const(27)
_IRQ_ENCRYPTION_UPDATE              = const(28)
_IRQ_GET_SECRET                     = const(29)
_IRQ_SET_SECRET                     = const(30)

# Advertising types (cf. https://docs.micropython.org/en/latest/library/ubluetooth.html)
_ADV_IND         = const(0x00)
_ADV_DIRECT_IND  = const(0x01)
_ADV_SCAN_IND    = const(0x02)
_ADV_NONCONN_IND = const(0x03)
_ADV_SCAN_RSP    = const(0x04)

# Address types (cf. https://docs.micropython.org/en/latest/library/ubluetooth.html)
_ADDR_TYPE_PUBLIC = const(0x00)
_ADDR_TYPE_RANDOM = const(0x01)

# Miflora Service / Characteristics UUIDs
# (ROOT_SERVICE could be used for discovery)
_GENERIC_ACCESS_SERVICE_UUID    = ubluetooth.UUID(0x1800)
_GENERIC_ATTRIBUTE_SERVICE_UUID = ubluetooth.UUID(0x1801)
_MIFLORA_ROOT_SERVICE_UUID      = ubluetooth.UUID('0000fe95-0000-1000-8000-00805f9b34fb')
_MIFLORA_DATA_SERVICE_UUID      = ubluetooth.UUID('00001204-0000-1000-8000-00805f9b34fb')
_MIFLORA_FIRM_CHAR_UUID         = ubluetooth.UUID('00001a02-0000-1000-8000-00805f9b34fb')
_MIFLORA_CMD_CHAR_UUID          = ubluetooth.UUID('00001a00-0000-1000-8000-00805f9b34fb')
_MIFLORA_DATA_CHAR_UUID         = ubluetooth.UUID('00001a01-0000-1000-8000-00805f9b34fb')

# Value handles and other magic spells specific to Miflora
_HANDLE_READ_VERSION_BATTERY = 0x38
_HANDLE_WRITE_MODE_CHANGE    = 0x33
_DATA_MODE_CHANGE            = bytes([0xA0, 0x1F])
_HANDLE_READ_SENSOR_DATA     = 0x35

# States of state machine
_S_INIT                = const(0)
_S_SCAN_DONE           = const(1)
_S_SERVICE_DONE        = const(2)
_S_CHARACTERISTIC_DONE = const(3)
_S_READ_FIRMWARE_DONE  = const(4)
_S_MODE_CHANGE_DONE    = const(5)
_S_READ_SENSOR_DONE    = const(6)

       
class MiFlora:
    """
    MiFlora Flower Care Bluetooth Low Energy (BLE) Driver

    Attributes:
        _ble (object):                  ubluetooth.BLE object (see BLE docs)
        state (int):                    state of MiFlora state machine
        search_addr (bytes):            BLE MAC address of device to search for
        addr_found (bool):              flag indicating whether device was found
        name (string):                  device name
        rssi (int):                     Received Signal Strength Indicator
        version (string):               MiFlora firmware version
        battery (int):                  battery status [%]
        temp (float):                   temperature [°C]
        light (int):                    light intensity [lx]
        moist (int):                    moisture [%]
        cond (int):                     conductivity [µS/cm]
        _addr_type (int):               address type (PUBLIC or RANDOM) (see BLE docs)
        _addr (bytes):                  BLE MAC address
        _value (memoryview):            cached data value (payload)
        _scan_callback (function):      callback for event _IRQ_SCAN_DONE
        _conn_callback (function):      callback for event _IRQ_PERIPHERAL_CONNECT
        _serv_done_callback (function): callback for event _IRQ_GATTC_SERVICE_DONE
        _char_done_callback (function): callback for event _IRQ_GATTC_CHARACTERISTIC_DONE
        _read_callback (function):      callback for event _IRQ_GATTC_READ_RESULT
        _write_callback (function):     callback for event _IRQ_GATTC_WRITE_DONE
        _notify_callback (function):    callback for event _IRQ_GATTC_NOTIFY
        _conn_handle (int):             connection handle
        _start_handle (int):            start handle (for characteristic discovery)
        _end_handle (int):              end   handle (for characteristic discovery)
        _value_handle (int):            value handle (for gattc_read()/gattc_write())
    """

    def __init__(self, ble):
        """
        The constructor for MiFlora class.

        Parameters:
            ble (ubluetooth.BLE):  ubluetooth.BLE object
        """
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        self._reset()


    def _reset(self):
        # Init public members.
        self.state       = _S_INIT
        self.search_addr = None
        self.addr_found  = False
        self.name        = None
        self.rssi        = 0
        self.version     = None
        self.battery     = None
        self.temp        = None
        self.light       = None
        self.moist       = None
        self.cond        = None

        
        # Cached name and address from a successful scan.
        self._addr_type  = None
        self._addr       = None

        # Cached value (if we have one).
        self._value      = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback      = None
        self._conn_callback      = None
        self._serv_done_callback = None
        self._char_done_callback = None
        self._read_callback      = None
        self._write_callback     = None
         
        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle     = None
        self._start_handle    = None
        self._end_handle      = None
        self._value_handle    = None

    def _debug(self, text, debuglevel = 0):
        """
        Debug output.

        Parameters:
            text (string):    string to be printed (including formatted variables, if desired)
            debuglevel (int): debuglevel must be less than or equal VERBOSITY, otherwise text will not be printed
        """
        if (debuglevel <= VERBOSITY):
            print(text)

    def _irq(self, event, data):
        """
        Interrupt request handler.

        See https://docs.micropython.org/en/latest/library/ubluetooth.html for description.

        Parameters:
            event (int):  interrupt request ID
            data (tuple): event specific data as tuple  
        """
        self._debug("bt_irq - event: {}".format(event), 3)
        
        if event == _IRQ_SCAN_RESULT:
            self._debug("bt irq - scan result", 2)
            # A single scan result.
            addr_type, addr, adv_type, rssi, adv_data = data
            _addr_type = 'Public' if (addr_type == _ADDR_TYPE_PUBLIC) else 'Random' 
            _addr = bytes(addr)
            _addr = binascii.hexlify(_addr)
            if adv_type == _ADV_IND:
                _adv_type = 'ADV_IND'
            elif adv_type == _ADV_DIRECT_IND:
                _adv_type = 'ADV_DIRECT_IND'
            elif adv_type == _ADV_SCAN_IND:
                _adv_type = 'ADV_SCAN_IND'
            elif adv_type == _ADV_NONCONN_IND:
                _adv_type = 'ADV_NONCONN_IND'
            else:
                _adv_type = 'SCAN_RSP'
            
            _adv_data = bytes(adv_data)
            _name = decode_name(_adv_data) or "?"
            _services = decode_services(adv_data)
            self._debug('addr_type: {}; addr: {}; adv_type: {}; rssi: {} dBm; name: {}; services: {}'.format(
                _addr_type, _addr, _adv_type, rssi, _name, _services), 1
            )

            if adv_type in (_ADV_IND, _ADV_DIRECT_IND, _ADV_SCAN_RSP) and bytes(addr) == self.search_addr:
                # Found a potential device, remember it and stop scanning.
                self._addr_type = addr_type
                self.rssi = rssi
                self.addr_found = True
                self._addr = bytes(
                    addr
                )  # Note: addr buffer is owned by caller so need to copy it.
                if _name != '?':
                    self.name = _name
                self._debug('Device name: {}'.format(_name), 1)
                self._ble.gap_scan(None)


        elif event == _IRQ_SCAN_DONE:
            self._debug("bt irq - scan done", 2)
            if self._scan_callback:
                if self._addr:
                    # Found a device during the scan (and the scan was explicitly stopped).
                    self._scan_callback(self._addr_type, self._addr, self.name)
                    self._scan_callback = None
                    if AUTO_MODE:
                        self.gap_connect(self._addr_type, self._addr)
                else:
                    # Scan timed out.
                    self._scan_callback(None, None, None)

        elif event == _IRQ_PERIPHERAL_CONNECT:
            # gap_connect() successful.
            self._debug("bt irq - peripheral connect", 2)
            conn_handle, addr_type, addr = data
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                if self._conn_callback:
                    self._conn_callback()
                    self._conn_callback = None
                if AUTO_MODE:
                    if _DISCOVER_SERVICES:
                        self.discover_services()
                    else:
                        self.read_firmware(callback=self.read_firmware_done)
        
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Disconnect (either initiated by us or the remote end).
            conn_handle, _, _ = data
            if conn_handle == self._conn_handle:
                # If it was initiated by us, it'll already be reset.
                self._reset()

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Connected device returned a service.
            self._debug("bt irq - gattc service result", 2)
            conn_handle, start_handle, end_handle, uuid = data
            
            if conn_handle == self._conn_handle:
                self._debug("{} -> service handle: {}...{}".format(uuid, start_handle, end_handle), 1)
                self.services[str(uuid)] = start_handle, end_handle
                if uuid == self.search_service:
                    self._debug("Wanted service {} has been discovered!".format(self.search_service), 1)
                    self._start_handle = start_handle
                    self._end_handle   = end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE:
            # Service query complete.
            self._debug("bt irq - gattc service done", 2)
            self.state = _S_SERVICE_DONE
            if self._serv_done_callback:
                self._serv_done_callback()
                self._serv_done_callback = None
            if AUTO_MODE:
                if _DISCOVER_CHARACTERISTICS:
                    # Note: In AUTO_MODE _start_handle/_end_handle should have been set according to desired service
                    #       in _IRQ_GATTC_SERVICE_RESULT.
                    if self._start_handle and self._end_handle:
                        self.discover_characteristics(self._start_handle, self._end_handle)
                    else:
                        self._debug("Failed to find gattc service.", 3)
                else:
                    self.read_firmware(callback=self.read_firmware_done)
                    
        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Connected device returned a characteristic.
            self._debug("bt irq - gattc characteristic result", 2)
            conn_handle, def_handle, value_handle, properties, uuid = data
            
            if conn_handle == self._conn_handle:
                self._debug("{}; def_handle: {}; value_handle: {}; properties: {}".format(
                    uuid, def_handle, value_handle, properties), 1
                )
                self.characteristics[str(uuid)] = def_handle, value_handle, properties

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Characteristic query complete.
            self._debug("bt irq - gattc characteristic done", 2)
            self.state = _S_CHARACTERISTIC_DONE
            if self._char_done_callback:
                self._char_done_callback()
                self._char_done_callback = None
            if AUTO_MODE:
                self.read_firmware(callback=self.read_firmware_done)

        elif event == _IRQ_GATTC_READ_RESULT:
            # A read completed successfully.
            self._debug("bt irq - gattc read result", 2)
            conn_handle, value_handle, char_data = data
            if conn_handle == self._conn_handle and value_handle == self._value_handle:
                self._update_value(char_data)
                if self._read_callback:
                    self._read_callback(self._value)
                    self._read_callback = None

        elif event == _IRQ_GATTC_READ_DONE:
            # Read completed (no-op).
            self._debug("bt irq - gattc read done", 2)
            conn_handle, value_handle, status = data
            if AUTO_MODE and self.state == _S_READ_FIRMWARE_DONE:
                self.mode_change(self.mode_change_done)

        elif event == _IRQ_GATTC_WRITE_DONE:
            # A gattc_write() has completed.
            # Note: The value_handle will be zero on btstack (but present on NimBLE).
            # Note: Status will be zero on success, implementation-specific value otherwise.
            self._debug("bt irq - gattc write done", 2)
            conn_handle, value_handle, status = data
            if conn_handle == self._conn_handle and value_handle == self._value_handle:
                self._debug("status: {}".format(status), 3)
                if self._write_callback:
                    self._write_callback()
                    self._write_callback = None
                if AUTO_MODE and self.state == _S_MODE_CHANGE_DONE:
                    self.read_sensor(callback=self.read_sensor_done)

        elif event == _IRQ_GATTC_NOTIFY:
            self._debug("bt irq - gattc notify", 2)
            
            conn_handle, value_handle, notify_data = data
            if conn_handle == self._conn_handle and value_handle == self._value_handle:
                self._update_value(notify_data)
                if self._notify_callback:
                    self._notify_callback(self._value)

    """
    Action trigger methods
    """
    def scan(self, callback=None):
        """
        Find all available devices.

        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gap_scan() parameters.

        Parameters:
            callback (function): callback to be invoked in _IRQ_SCAN_DONE if the desired device
                                 was found in _IRQ_SCAN_RESULT
        """
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        try:
            self._ble.gap_scan(2000, 30000, 30000, True)
        except OSError as e:
            pass

    def gap_connect(self, addr_type=None, addr=None, callback=None):
        """
        Connect to the specified device (otherwise use cached address from a scan).

        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gap_connect().

        Parameters:
            addr_type (int):     address type (PUBLIC or RANDOM)
            addr (bytes):        BLE MAC address
            callback (function): callback to be invoked in _IRQ_PERIPHERAL_CONNECT
            
        Returns:
            bool: True  if valid address type and address was available and gap_connect() was called without error
                        (not connected yet!),
                  False otherwise
        """
        self._debug("gap_connect()", 1)
        if not(addr_type is None) and not(addr is None):
            # if provided, use address type and address provided as parameters
            # (otherwise use address type and address from preceeding scan)
            self._addr_type = addr_type
            self._addr = addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            self.debug("gap_connect(): Parameter error! _addr_type: {}; _addr: {}".format(
                self._addr_type, self._addr), 1
            )
            return False
        try:
            self._ble.gap_connect(self._addr_type, self._addr)
            return True
        except OSError as e:
            return False
        
    def disconnect(self):
        """
        Disconnect from current device and reset object's attributes.
        """
        self._debug("disconnect()", 1)
        if not self._conn_handle:
            return
        try:
            self._ble.gap_disconnect(self._conn_handle)
        except OSError as e:
            pass
        self._reset()

    def discover_services(self, callback=None):
        """
        Discover services provided by connected device.

        All discovered services are stored in 'services'.
        For if service with UUID provided in 'search_service' was discovered,
        '_start_handle' and '_end_handle' for this service are stored.
        
        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gattc_discover_services().
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_SERVICE_DONE
        """
        self._debug("discover_services()", 1)
        self.services = {}
        if not self.is_connected():
            return
        self._serv_done_callback = callback
        try:
            self._ble.gattc_discover_services(self._conn_handle)
        except OSError as e:
            pass

    def discover_characteristics(self, start_handle, end_handle, callback=None):
        """
        Discover characteristics of connected device in range specified by start_handle/end_handle.

        All discovered characteristics are stored in 'characteristics'.
        
        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gattc_discover_services().
        
        Parameters:
            start_handle (int):  start of characteristic range
            end_handle (int):    end of characteristic range
            callback (function): callback to be invoked in _IRQ_GATTC_CHARACTERISTIC_DONE
        """
        self._debug("discover_characteristics()", 1)
        self.characteristics = {}
        if not self.is_connected():
            return
        self._char_done_callback = callback
        try:
            self._ble.gattc_discover_characteristics(self._conn_handle, start_handle, end_handle)
        except OSError as e:
            pass
        
    def read(self, callback):
        """
        Generic read access.
        
        Issues an (asynchronous) read of _value_handle, will invoke callback with data.
        
        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gattc_read()
        
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_READ_RESULT
        """
        self._debug("read()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass

    def read_firmware(self, callback):
        """
        Read firmware version and battery status.
        
        Issues an (asynchronous) read from _value_handle = _HANDLE_READ_VERSION_BATTERY, 
        will invoke callback with data.
        
        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gattc_read()
        
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_READ_RESULT
        """
        self._debug("read_firmware()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback
        self._value_handle = _HANDLE_READ_VERSION_BATTERY
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass
   
    def mode_change(self, callback):
        """
        Change data mode to allow reading of sensor data.
        
        Issues an (asynchronous) write to _value_handle = _DATA_MODE_CHANGE, 
        will invoke callback after completion.
        
        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gattc_write()
        
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_WRITE_DONE
        """
        self._debug("mode_change()", 1)
        self._write_callback = callback
        self._value_handle = _HANDLE_WRITE_MODE_CHANGE
        try:
            self._ble.gattc_write(self._conn_handle, self._value_handle, _DATA_MODE_CHANGE, 1)
        except OSError as e:
            pass
    
    def read_sensor(self, callback):
        """
        Read sensor data.
        
        Issues an (asynchronous) read from _value_handle = _HANDLE_READ_SENSOR_DATA, 
        will invoke callback with data.
        
        See https://docs.micropython.org/en/latest/library/ubluetooth.html for gattc_write()
        
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_READ_RESULT
        """
        self._debug("read_sensor()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback
        self._value_handle = _HANDLE_READ_SENSOR_DATA
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass
    
    """
    Callback methods
    """
    def scan_done(self, addr_type, addr, name):
        """
        Callback for scan().
        
        The parameters are those of the device which matched the search criterion.
        (In this case the address.)
        
        Parameters:
            addr_type (int): address type (PUBLIC or RANDOM)
            addr (bytes):    BLE MAC address
            name (string):   device name
        """
        self._debug("scan_done()", 1)
        self.state = _S_SCAN_DONE

    def read_firmware_done(self, data):
        """
        Callback for read_firmware().
        
        The battery level and the firmware version are copied from the read data.
        
        Parameters:
            data (memoryview): read data
        """        
        self._debug("read_firmware_done()", 1)
        data = bytes(data)
        self.battery = data[0]
        self.version = str(data[2:7], 'UTF-8')
        self.state = _S_READ_FIRMWARE_DONE

    def mode_change_done(self):
        """
        Callback for mode_change().
        """        
        self._debug("mode_change_done()", 1)
        self.state = _S_MODE_CHANGE_DONE

    def read_sensor_done(self, data):
        """
        Callback for read_sensor().
        
        The sensor data - temperature, light intensity, moisture and conductivity - are copied from the read data.
        
        Parameters:
            data (memoryview): read data
        """        
        self._debug("read_sensor_done()", 1)
        data = bytes(data)
        self._debug("data(): {}".format(data), 3)
        # Note 1: ustruct.unpack() does not support padding
        #         (cf. https://docs.micropython.org/en/latest/library/ustruct.html)
        # Note 2: (u)struct() always returns a tuple, even if the result is a single element
        temp  = struct.unpack("<h", data[0:2])
        light = struct.unpack("<I", data[3:7])
        moist = struct.unpack("<B", data[7:8])
        cond  = struct.unpack("<h", data[8:10])
        self.temp  = temp[0]/10.0
        self.light = light[0]
        self.moist = moist[0]
        self.cond  = cond[0]
        self.state = _S_READ_SENSOR_DONE

    """
    Status query methods
    """
    def is_connected(self):
        """
        Check if connected to device.

        Returns:
            bool: True  if connected,
                  False otherwise.
        """
        return self._conn_handle is not None

    def wait_for_connection(self, status, timeout_ms):
        """
        Wait until connection reaches 'status' or a timeout occurrs.

        The connection status is polled in _T_WAIT intervals.

        Parameters:
            status (bool):     expected connection status
            timeout_ms (int) : timeout in ms

        Returns:
            bool: True  desired status occurred,
                  False timeout ocurred.
        """
        t0 = time.ticks_ms()
        
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            if self.is_connected() == status:
                return True
            time.sleep_ms(_T_WAIT)
        return False

    def wait_for(self, state, timeout_ms):
        """
        Wait until 'state' is reached or a timeout occurrs.

        The state is polled in _T_WAIT intervals.

        Parameters:
            status (bool):     expected connection status
            timeout_ms (int) : timeout in ms

        Returns:
            bool: True  desired status occurred,
                  False timeout ocurred.
        """
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            if self.state == state:
                return True
            time.sleep_ms(_T_WAIT)
        return False

    """
    Helper methods
    """
    def on_notify(self, callback):
        """
        Set a callback for device notifications.
                
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_NOTIFY
        """        
        self._debug("on_notify()", 1)
        self._notify_callback = callback

    def _update_value(self, data):
        """
        Update value from a notification or a read access.

        Parameters:
            data (memoryview): notification/read data (payload)
            
        Returns:
            memoryview: object in memory containing payload data
        """
        self._debug("_update_value()", 2)
        self._value = data
        return self._value
            
    def value(self):
        """
        Read access function for '_value'. (?) 
        """
        return self._value
### END class MiFlora


# Stand-alone version of MiFlora.wait_for() - obsolete!
def wait_for(obj, state, timeout_ms):

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if getattr(obj, 'state') == state:
            return True
        time.sleep_ms(_T_WAIT)
    return False

# Stand-alone version of MiFlora.wait_for_connection() - obsolete!
def wait_for_connection(obj, status, timeout_ms):
    t0 = time.ticks_ms()
    
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if obj.is_connected() == status:
            return True
        time.sleep_ms(_T_WAIT)
    return False


###############################################################################
# demo_man() - test function for class MiFlora ("manual" mode)
#
# Sensor access is controlled step-by-step from the application. Typically an
# action is trigged, a callback function is invoked upon completion and the 
# application waits until the expected state of the MiFlora class is reached
# (or a timeout occurred).
#
# This is example might be useful as a base for integrating other BLE devices.  
###############################################################################
def demo_man():
    ble = ubluetooth.BLE()
    miflora = MiFlora(ble)
    print("demo_man()")
    
    while True:
        for addr in miflora_sensors:
            miflora.search_addr    = addr
            miflora.search_service = _GENERIC_ACCESS_SERVICE_UUID

            if SCAN_DEVICES:
                print("Searching for device with MAC address {}...".format(binascii.hexlify(addr)))
                
                miflora.scan(callback=miflora.scan_done)
                
                if miflora.wait_for(_S_SCAN_DONE, 2500):
                    print("Scan done.")
                else:
                    print("Scan timeout!")
                    continue
                    
                if miflora.addr_found == False:
                    print("Sensor not found!")
                    continue
                
                print("Sensor '{}' found.".format(miflora.name))
                print("RSSI: {}dbB".format(miflora.rssi))
        
            print("Trying to connect to device with MAC address {}...".format(binascii.hexlify(addr)))
            rc = miflora.gap_connect(_ADDR_TYPE_PUBLIC, addr)
            print("gap_connect() = ", rc)
            
            if miflora.wait_for_connection(True, 3000):
                print("Connected")
            else:
                print("Connection failed!")
                continue
            
            if DISCOVER_SERVICES:
                miflora.discover_services()
                
                if miflora.wait_for(_S_SERVICE_DONE, 2500):
                    print(miflora.services)
                else:
                    print("discover_services failed!")
                
            if DISCOVER_CHARACTERISTICS:
                miflora.discover_characteristics(1, 9)
                
                if miflora.wait_for(_S_CHARACTERISTIC_DONE, 2500):
                    print(miflora.characteristics)
                else:
                    print("discover_characteristics failed!")
            
            miflora.read_firmware(callback=miflora.read_firmware_done)
        
            if miflora.wait_for(_S_READ_FIRMWARE_DONE, 2000):
                print("Battery Level: {}%".format(miflora.battery))
                print("Version: {}".format(miflora.version))
        
                miflora.mode_change(miflora.mode_change_done)
        
                if not miflora.wait_for(_S_MODE_CHANGE_DONE, 2000):
                    print("Mode change failed!")
                    break;
            
                miflora.read_sensor(callback=miflora.read_sensor_done)
                
                if miflora.wait_for(_S_READ_SENSOR_DONE, 2000):
                     print("Temperature: {}°C Light: {}lx Moisture: {}% Conductivity: {}µS/cm".format(
                        miflora.temp, miflora.light, miflora.moist, miflora.cond)
                    )
                else:
                    print("Reading sensor data failed!")
            else:
                print("Reading sensor firmware version and battery status failed!")

            miflora.disconnect()
            
            if miflora.wait_for_connection(False, 10000):
                print("Disconnected")
            else:
                print("Disconnect failed!")
                miflora._reset()

        print("Sleeping {} seconds...".format(_T_CYCLE))
        time.sleep(_T_CYCLE)


###############################################################################
# demo_auto() - test function for class MiFlora (auto mode)
#
# Sensor access is initiated and runs in background until # completed
# (or a timeout ocurred).
#
# This is the preferred example for an application using MiFlora sensors.  
###############################################################################
def demo_auto():
    ble = ubluetooth.BLE()
    miflora = MiFlora(ble)
    print("demo_auto()")

    while True:
        
        for addr in miflora_sensors:
            miflora.search_addr    = addr
            miflora.search_service = _GENERIC_ACCESS_SERVICE_UUID

            if SCAN_DEVICES:
                print("Searching for device with MAC address {}...".format(binascii.hexlify(addr)))
                
                miflora.scan(callback=miflora.scan_done)
            else:
                print("Trying to connect to device with MAC address {}...".format(binascii.hexlify(addr)))
                
                rc = miflora.gap_connect(_ADDR_TYPE_PUBLIC, addr)
                print("gap_connect() = ", rc)
                
            ########################################
            # Time to perform other tasks...
            ########################################
            
            # The time required depends on the number of visible devices and
            # the selected service discovery/characteristics discovery options.
            if miflora.wait_for(_S_READ_SENSOR_DONE, 20000):
                if SCAN_DEVICES:
                    print("Sensor '{}' found.".format(miflora.name))
                    print("RSSI: {}dbB".format(miflora.rssi))
                print("Battery Level: {}%".format(miflora.battery))
                print("Version: {}".format(miflora.version))
                print("Temperature: {}°C Light: {}lx Moisture: {}% Conductivity: {}µS/cm".format(
                    miflora.temp, miflora.light, miflora.moist, miflora.cond)
                )
            else:
                print("Reading sensor data failed (timeout)!")

            miflora.disconnect()
            
            if miflora.wait_for_connection(False, 10000):
                print("Disconnected")
            else:
                print("Disconnect failed (timeout)!")
                miflora._reset()

        print("Sleeping {} seconds...".format(_T_CYCLE))
        time.sleep(_T_CYCLE)


if __name__ == "__main__":
    if AUTO_MODE:
        demo_auto()
    else:
        demo_man()
