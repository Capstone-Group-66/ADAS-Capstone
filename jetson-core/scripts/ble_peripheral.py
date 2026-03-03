#!/usr/bin/env python3
"""
BLE Peripheral for ADAS Pipeline (Python 3.6 Compatible)
---------------------------------------------------------
This script is spawned by the C++ SimpleBleServer and receives
CBOR packets via stdin to send over BLE.

Uses BlueZ D-Bus for BLE GATT server (works on old Jetson).

Protocol:
    C++ sends JSON lines to stdin:
    {"cmd":"notify","data":"<hex_encoded_cbor>"}
    
    This script decodes the hex and sends via BLE notification.
"""

import sys
import json
import logging
import threading
import struct

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [BLE] %(message)s',
    datefmt='%H:%M:%S',
    stream=sys.stderr
)
logger = logging.getLogger("BLE")

# Try to import D-Bus components
try:
    import dbus
    import dbus.service
    import dbus.mainloop.glib
    from gi.repository import GLib
    HAS_DBUS = True
except ImportError as e:
    logger.warning(f"D-Bus not available: {e}")
    HAS_DBUS = False

# BLE Constants
BLUEZ_SERVICE = 'org.bluez'
ADAPTER_IFACE = 'org.bluez.Adapter1'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

ADAS_SERVICE_UUID = '0000ada5-0000-1000-8000-00805f9b34fb'
ADAS_ALERT_STREAM_UUID = '0000a1e7-0000-1000-8000-00805f9b34fb'
ADAS_COMMAND_UUID = '0000c0ad-0000-1000-8000-00805f9b34fb'

# Try to import CBOR decoder for GPS data
try:
    import cbor2
    HAS_CBOR = True
except ImportError:
    logger.warning("cbor2 not available: pip3 install cbor2")
    HAS_CBOR = False


class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/adas/advertisement'

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = 'peripheral'
        self.local_name = 'ADAS-Jetson'
        self.service_uuids = [ADAS_SERVICE_UUID]
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            LE_ADVERTISEMENT_IFACE: {
                'Type': self.ad_type,
                'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s'),
                'LocalName': dbus.String(self.local_name),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise dbus.exceptions.DBusException(
                'org.freedesktop.DBus.Error.InvalidArgs',
                'Invalid interface')
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature='', out_signature='')
    def Release(self):
        logger.info('Advertisement released')


class Service(dbus.service.Object):
    PATH_BASE = '/org/bluez/adas/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
                'Characteristics': dbus.Array(
                    [c.get_path() for c in self.characteristics],
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, chrc):
        self.characteristics.append(chrc)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise dbus.exceptions.DBusException(
                'org.freedesktop.DBus.Error.InvalidArgs',
                'Invalid interface')
        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.notifying = False
        self.value = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
                'Notifying': self.notifying,
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise dbus.exceptions.DBusException(
                'org.freedesktop.DBus.Error.InvalidArgs',
                'Invalid interface')
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        logger.info("Client subscribed to notifications")

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
        logger.info("Client unsubscribed from notifications")

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def send_notification(self, data):
        if not self.notifying:
            logger.debug("No subscribers, notification skipped")
            return False
        self.value = dbus.Array(data, signature='y')
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': self.value}, [])
        return True


class CommandCharacteristic(Characteristic):
    """Writable characteristic for phone -> Jetson data (GPS, commands).
    
    Wire format: [type_byte][payload...]
    Type 0x01 = GPS data (CBOR-encoded GpsData from Android)
    
    Decoded GPS is printed to stdout as JSON for C++ to read.
    """

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index, ADAS_COMMAND_UUID,
            ['write-without-response'], service)

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        data = bytes(value)
        if len(data) < 2:
            logger.warning("Command write too short: %d bytes", len(data))
            return

        msg_type = data[0]
        payload = data[1:]

        if msg_type == 0x01:  # GPS
            self._handle_gps(payload)
        else:
            logger.warning("Unknown command type: 0x%02x", msg_type)

    def _handle_gps(self, payload):
        """Decode CBOR GPS data and emit JSON to stdout."""
        if not HAS_CBOR:
            logger.error("GPS received but cbor2 not installed")
            return
        try:
            gps = cbor2.loads(payload)
            out = json.dumps({
                "event": "gps",
                "speed_mps": gps.get("speedMps", 0.0),
                "ts_ms": gps.get("tsMs", 0)
            })
            sys.stdout.write(out + "\n")
            sys.stdout.flush()
            logger.debug("GPS: speed=%.1f m/s", gps.get("speedMps", 0.0))
        except Exception as e:
            logger.error("GPS CBOR decode error: %s", e)


class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = '/org/bluez/adas'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for chrc in service.characteristics:
                response[chrc.get_path()] = chrc.get_properties()
        return response


class BlePeripheral:
    def __init__(self):
        if HAS_DBUS:
            dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
            self.bus = dbus.SystemBus()
            self.mainloop = GLib.MainLoop()
        self.alert_chrc = None
        self.cmd_chrc = None
        self.running = True
        
    def find_adapter(self):
        remote_om = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, '/'),
            DBUS_OM_IFACE)
        objects = remote_om.GetManagedObjects()
        for o, props in objects.items():
            if GATT_MANAGER_IFACE in props:
                return o
        return None

    def setup(self):
        if not HAS_DBUS:
            logger.info("Running in stub mode (no D-Bus)")
            return True
            
        adapter_path = self.find_adapter()
        if not adapter_path:
            logger.error("No BlueZ adapter found!")
            return False
            
        logger.info("Using adapter: %s", adapter_path)
        
        self.app = Application(self.bus)
        service = Service(self.bus, 0, ADAS_SERVICE_UUID, True)
        self.alert_chrc = Characteristic(
            self.bus, 0, ADAS_ALERT_STREAM_UUID,
            ['notify'], service
        )
        service.add_characteristic(self.alert_chrc)
        
        # Command characteristic: phone -> Jetson (GPS data, future commands)
        self.cmd_chrc = CommandCharacteristic(self.bus, 1, service)
        service.add_characteristic(self.cmd_chrc)
        
        self.app.add_service(service)
        
        self.adv = Advertisement(self.bus, 0)
        
        gatt_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path),
            GATT_MANAGER_IFACE)
        
        try:
            gatt_manager.RegisterApplication(
                self.app.get_path(), {},
                reply_handler=lambda: logger.info("GATT registered"),
                error_handler=lambda e: logger.error("GATT failed: %s", e)
            )
        except Exception as e:
            logger.error("RegisterApplication failed: %s", e)
            return False
        
        ad_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path),
            LE_ADVERTISING_MANAGER_IFACE)
        
        try:
            ad_manager.RegisterAdvertisement(
                self.adv.get_path(), {},
                reply_handler=lambda: logger.info("Advertising started"),
                error_handler=lambda e: logger.error("Advertising failed: %s", e)
            )
        except Exception as e:
            logger.error("RegisterAdvertisement failed: %s", e)
            return False
            
        return True

    def send_notification(self, data_bytes):
        """Send binary data as BLE notification."""
        if self.alert_chrc:
            if self.alert_chrc.send_notification(list(data_bytes)):
                logger.info("Sent %d bytes via BLE", len(data_bytes))
                return True
        logger.info("Would send %d bytes (stub mode or no subscribers)", len(data_bytes))
        return False

    def process_stdin_command(self, line):
        """Process a JSON command from C++ stdin."""
        try:
            cmd = json.loads(line.strip())
            
            if cmd.get("cmd") == "notify":
                hex_data = cmd.get("data", "")
                data = bytes.fromhex(hex_data)
                self.send_notification(data)
            else:
                logger.warning("Unknown command: %s", cmd)
                
        except json.JSONDecodeError as e:
            logger.error("Invalid JSON: %s", e)
        except Exception as e:
            logger.error("Error processing command: %s", e)

    def stdin_reader_thread(self):
        """Read commands from stdin in a separate thread."""
        logger.info("Listening for commands on stdin...")
        
        while self.running:
            try:
                line = sys.stdin.readline()
                if not line:
                    logger.info("EOF on stdin, exiting")
                    self.running = False
                    if HAS_DBUS:
                        GLib.idle_add(self.mainloop.quit)
                    break
                    
                # Schedule command processing on main thread
                if HAS_DBUS:
                    GLib.idle_add(self.process_stdin_command, line)
                else:
                    self.process_stdin_command(line)
                    
            except Exception as e:
                logger.error("Stdin read error: %s", e)
                break

    def run(self):
        logger.info("=" * 50)
        logger.info("ADAS BLE Peripheral (Pipeline Mode)")
        logger.info("Service: %s", ADAS_SERVICE_UUID)
        logger.info("=" * 50)
        
        if not self.setup():
            logger.error("Setup failed!")
            return
        
        # Start stdin reader thread
        stdin_thread = threading.Thread(target=self.stdin_reader_thread, daemon=True)
        stdin_thread.start()
        
        if HAS_DBUS:
            try:
                self.mainloop.run()
            except KeyboardInterrupt:
                logger.info("Interrupted")
                self.mainloop.quit()
        else:
            # Stub mode: just process stdin
            stdin_thread.join()


if __name__ == '__main__':
    peripheral = BlePeripheral()
    peripheral.run()
