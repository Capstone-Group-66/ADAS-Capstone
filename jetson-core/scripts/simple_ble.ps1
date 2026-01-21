# Simple BLE Server (PowerShell 5.1 compatible - Polling)
# Uses explicit WinRT Type Loading

function Get-WinRTType($FullName, $DllName) {
    # Try simple load
    $type = $null
    try { $type = [$FullName] } catch {}
    if ($type) { return $type }

    # Try ContentType=WindowsRuntime
    $str = "$FullName, $DllName, ContentType=WindowsRuntime"
    $type = [Type]::GetType($str)
    if (-not $type) {
        Throw "Could not load WinRT type: $FullName"
    }
    return $type
}

# Helper to wait for Async Operation
function Wait-AsyncOp($Operation, $TimeoutSeconds = 10) {
    if (-not $Operation) { Throw "Operation is null" }
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($Operation.Status -eq "Started") {
        if ($sw.Elapsed.TotalSeconds -gt $TimeoutSeconds) {
            Throw "Async Operation Timed Out"
        }
        Start-Sleep -Milliseconds 100
    }
    
    if ($Operation.Status -eq "Error") {
        Throw "Async Operation Failed: $($Operation.ErrorCode)"
    }
    
    # Try calling GetResults
    return $Operation.GetResults()
}

$ErrorActionPreference = "Stop"

try {
    Write-Host "Initializing BLE Server (WinRT)..."

    # Load Types
    $GattServiceProviderType = Get-WinRTType "Windows.Devices.Bluetooth.GenericAttributeProfile.GattServiceProvider" "Windows.Devices.Bluetooth"
    $GattLocalCharacteristicParametersType = Get-WinRTType "Windows.Devices.Bluetooth.GenericAttributeProfile.GattLocalCharacteristicParameters" "Windows.Devices.Bluetooth"
    $GattServiceProviderAdvertisingParametersType = Get-WinRTType "Windows.Devices.Bluetooth.GenericAttributeProfile.GattServiceProviderAdvertisingParameters" "Windows.Devices.Bluetooth"
    $CryptographicBufferType = Get-WinRTType "Windows.Security.Cryptography.CryptographicBuffer" "Windows.Security.Cryptography"

    # UUIDs
    $ServiceUuid = [Guid]"0000ada5-0000-1000-8000-00805f9b34fb"
    $AlertUuid   = [Guid]"0000a1e7-0000-1000-8000-00805f9b34fb"

    # 1. Create Service Provider
    Write-Host "Creating Service Provider..."
    # Invoke static method on Type object
    $op = $GattServiceProviderType::CreateAsync($ServiceUuid)
    $providerResult = Wait-AsyncOp $op
    
    if ($providerResult.Error -ne "Success") {
        Throw "Failed to create service: $($providerResult.Error)"
    }
    $provider = $providerResult.ServiceProvider
    $service = $provider.Service
    Write-Host "Service Created: $ServiceUuid"

    # 2. Create Characteristic
    Write-Host "Creating Characteristic..."
    # Constructor on Type object? 
    # [Activator]::CreateInstance($GattLocalCharacteristicParametersType)
    $charParams = [Activator]::CreateInstance($GattLocalCharacteristicParametersType)
    
    # Enum handling is tricky. We can use int or string cast?
    # Notify = 16 (0x10) from metadata?
    # Or try finding the Enum type.
    # GattCharacteristicProperties is enum.
    # We can just pass the integer 16? 
    # Or string "Notify" if property allows.
    # WinRT enums are tricky.
    
    # We will assume integer works or property setting works.
    # But Wait, PS is dynamically typed for COM.
    # $charParams.CharacteristicProperties = "Notify" might work if PS helper exists.
    # IF NOT: We send integer. Notify = 16.
    $charParams.CharacteristicProperties = 16 
    $charParams.UserDescription = "AlertStream"
    
    $op = $service.CreateCharacteristicAsync($AlertUuid, $charParams)
    $charResult = Wait-AsyncOp $op
    
    if ($charResult.Error -ne "Success") {
        Throw "Failed to create characteristic: $($charResult.Error)"
    }
    $characteristic = $charResult.Characteristic
    Write-Host "Characteristic Created: $AlertUuid"

    # 3. Start Advertising
    Write-Host "Starting Advertisement..."
    $advParams = [Activator]::CreateInstance($GattServiceProviderAdvertisingParametersType)
    $advParams.IsConnectable = $true
    $advParams.IsDiscoverable = $true
    
    $provider.StartAdvertising($advParams)
    Write-Host "Advertising started!"

    # 4. Loop
    $tick = 0
    while ($true) {
        $tick++
        if ($tick -gt 65535) { $tick = 0 }
        
        Write-Host "Sending Alert (Tick $tick)... " -NoNewline

        # Build Packet (CBOR + Header)
        $tickHi = [byte]($tick / 256)
        $tickLo = [byte]($tick % 256)
        
        $packet = @(
            $tickLo, $tickHi, 0x00, 0x00, # Header
            0xA5,                         # Map(5)
            0x61, 0x74, 0x19, $tickHi, $tickLo,
            0x61, 0x76, 0x18, 0x3C,
            0x61, 0x68, 0x00,
            0x61, 0x62, 0x00,
            0x61, 0x61, 0x81, 0xA3, 0x62, 0x69, 0x64, 0x00, 0x61, 0x73, 0x01, 0x61, 0x72, 0x64, 0x54, 0x65, 0x73, 0x74
        )
        
        # Convert to IBuffer
        $byteArray = [byte[]]$packet
        $buffer = $CryptographicBufferType::CreateFromByteArray($byteArray)

        # Notify
        try {
            $op = $characteristic.NotifyValueAsync($buffer)
            $res = Wait-AsyncOp $op 2
            Write-Host "Sent."
        } catch {
            Write-Host "Error sending: $_"
        }
        
        Start-Sleep -Seconds 3
    }

} catch {
    Write-Error "Error: $_"
    Write-Error $($_.ScriptStackTrace)
}
