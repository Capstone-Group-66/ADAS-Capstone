using System;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Foundation;
using Windows.Storage.Streams;
using Windows.Security.Cryptography;
using System.Runtime.InteropServices.WindowsRuntime; // Helper for extensions

class Program {
    static void Main(string[] args) {
        MainAsync(args).Wait();
    }

    static async Task MainAsync(string[] args) {
        Console.WriteLine("Starting Native BLE Server (C#)...");
        
        var serviceUuid = new Guid("0000ada5-0000-1000-8000-00805f9b34fb");
        var alertUuid = new Guid("0000a1e7-0000-1000-8000-00805f9b34fb");
        
        try {
            // Create Service
            Console.WriteLine("Creating Service...");
            var providerResult = await GattServiceProvider.CreateAsync(serviceUuid);
            if (providerResult.Error != BluetoothError.Success) {
                Console.WriteLine("Error creating service: " + providerResult.Error);
                return;
            }
            var provider = providerResult.ServiceProvider;
            
            // Create Characteristic
            Console.WriteLine("Creating Characteristic...");
            var charParams = new GattLocalCharacteristicParameters {
                CharacteristicProperties = GattCharacteristicProperties.Notify,
                UserDescription = "AlertStream"
            };
            var charResult = await provider.Service.CreateCharacteristicAsync(alertUuid, charParams);
            if (charResult.Error != BluetoothError.Success) {
                Console.WriteLine("Error creating char: " + charResult.Error);
                return;
            }
            var characteristic = charResult.Characteristic;
            
            // Start Advertising
            Console.WriteLine("Starting Advertising...");
            var advParams = new GattServiceProviderAdvertisingParameters {
                IsConnectable = true,
                IsDiscoverable = true
            };
            provider.StartAdvertising(advParams);
            Console.WriteLine("Advertising started. Waiting for connections...");
            
            // Payload
            // Header: 39 30 00 00
            // CBOR: a561...
            string hex = "39300000a561741930396176183c616801616200616181a3626964006173026172684643572054657374";
            byte[] bytes = new byte[hex.Length / 2];
            for (int i = 0; i < hex.Length; i += 2)
                bytes[i / 2] = Convert.ToByte(hex.Substring(i, 2), 16);
                
            var buffer = CryptographicBuffer.CreateFromByteArray(bytes);
            
            while (true) {
                await Task.Delay(3000);
                try {
                     Console.Write("Sending... ");
                     await characteristic.NotifyValueAsync(buffer);
                     Console.WriteLine("Sent.");
                } catch (Exception ex) {
                    Console.WriteLine("Error: " + ex.Message);
                }
            }
        } catch (Exception e) {
            Console.WriteLine("Exception: " + e.ToString());
        }
    }
}
