#!/usr/bin/env python3
"""
FCW Simulator for Jetson Nano (Python 3.6) - BlueZ D-Bus Implementation
------------------------------------------------------------------------
Uses dbus-python and GLib mainloop for BLE GATT server.
This works on older Jetson systems with Python 3.6.

Install: sudo apt install python3-dbus python3-gi
"""

import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib
import struct
import logging
import sys
import threading
import time

try:
    import cbor
except ImportError:
    print("Missing cbor. Run: pip3 install cbor")
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("FCW_Sim")

# BlueZ D-Bus constants
BLUEZ_SERVICE = 'org.bluez'
ADAPTER_IFACE = 'org.bluez.Adapter1'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

# ADAS UUIDs
ADAS_SERVICE_UUID = '0000ada5-0000-1000-8000-00805f9b34fb'
ADAS_ALERT_STREAM_UUID = '0000a1e7-0000-1000-8000-00805f9b34fb'

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
        properties = {
            LE_ADVERTISEMENT_IFACE: {
                'Type': self.ad_type,
                'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s'),
                'LocalName': dbus.String(self.local_name),
            }
        }
        return {LE_ADVERTISEMENT_IFACE: properties[LE_ADVERTISEMENT_IFACE]}

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

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}', out_signature='')
    def WriteValue(self, value, options):
        self.value = value

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        logger.info(f"Notifications started for {self.uuid}")

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
        logger.info(f"Notifications stopped for {self.uuid}")

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def send_notification(self, data):
        if not self.notifying:
            return False
        self.value = dbus.Array(data, signature='y')
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': self.value}, [])
        return True


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


class FcwSimulator:
    def __init__(self):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.mainloop = GLib.MainLoop()
        self.tick_id = 0
        self.alert_chrc = None
        
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
        adapter_path = self.find_adapter()
        if not adapter_path:
            logger.error("No BlueZ adapter found!")
            return False
            
        logger.info(f"Using adapter: {adapter_path}")
        
        # Create application
        self.app = Application(self.bus)
        
        # Create ADAS service
        service = Service(self.bus, 0, ADAS_SERVICE_UUID, True)
        
        # Create AlertStream characteristic
        self.alert_chrc = Characteristic(
            self.bus, 0, ADAS_ALERT_STREAM_UUID,
            ['notify'], service
        )
        service.add_characteristic(self.alert_chrc)
        self.app.add_service(service)
        
        # Create advertisement
        self.adv = Advertisement(self.bus, 0)
        
        # Register application
        gatt_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path),
            GATT_MANAGER_IFACE)
        
        try:
            gatt_manager.RegisterApplication(
                self.app.get_path(), {},
                reply_handler=lambda: logger.info("GATT application registered"),
                error_handler=lambda e: logger.error(f"Failed to register: {e}")
            )
        except Exception as e:
            logger.error(f"RegisterApplication failed: {e}")
            return False
        
        # Register advertisement
        ad_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path),
            LE_ADVERTISING_MANAGER_IFACE)
        
        try:
            ad_manager.RegisterAdvertisement(
                self.adv.get_path(), {},
                reply_handler=lambda: logger.info("Advertisement registered"),
                error_handler=lambda e: logger.error(f"Failed to advertise: {e}")
            )
        except Exception as e:
            logger.error(f"RegisterAdvertisement failed: {e}")
            return False
            
        return True

    def generate_fcw_payload(self):
        self.tick_id = (self.tick_id + 1) % 65536
        
        payload_data = {
            "t": self.tick_id,
            "v": 65,
            "h": 0,
            "b": 0,
            "a": [{"id": 0, "s": 2, "r": "Simulated FCW"}]
        }
        
        cbor_bytes = cbor.dumps(payload_data)
        header = struct.pack('<HBB', self.tick_id, 0, 0)
        return header + cbor_bytes

    def send_alert(self):
        packet = self.generate_fcw_payload()
        if self.alert_chrc:
            if self.alert_chrc.send_notification(list(packet)):
                logger.info(f"Sent FCW Tick #{self.tick_id}")
            else:
                logger.info(f"Generated Tick #{self.tick_id} (no subscribers)")
        return True  # Keep timer running

    def run(self):
        logger.info("=" * 60)
        logger.info("  ADAS FCW Simulator (BlueZ D-Bus)")
        logger.info("=" * 60)
        
        if not self.setup():
            logger.error("Setup failed!")
            return
        
        # Schedule FCW alerts every 10 seconds
        GLib.timeout_add_seconds(10, self.send_alert)
        
        logger.info("Server running. Press Ctrl+C to stop.")
        
        try:
            self.mainloop.run()
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            self.mainloop.quit()


if __name__ == '__main__':
    sim = FcwSimulator()
    sim.run()
