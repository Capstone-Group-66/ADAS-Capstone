# IMPORTS
Add-Type -AssemblyName System.Runtime.WindowsRuntime
Add-Type -AssemblyName System.Runtime.InteropServices.WindowsRuntime

Function Await($AsyncOp) {
    # Find generic AsTask<T>(IAsyncOperation<T>)
    # It must be Generic and take 1 parameter.
    $methods = [System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object { $_.Name -eq "AsTask" }
    
    $method = $methods | Where-Object { 
        $_.IsGenericMethod -and $_.GetParameters().Count -eq 1
    } | Select-Object -First 1

    if ($method -eq $null) {
        Write-Error "Could not find AsTask method."
        return $null
    }

    # Get T from the operation
    # The object implements IAsyncOperation<T>
    # We find the interface that is generic IAsyncOperation
    $interface = $AsyncOp.GetType().GetInterfaces() | Where-Object { 
        $_.Name -eq "IAsyncOperation`1" 
    } | Select-Object -First 1
    
    if ($interface -eq $null) {
        Write-Error "Object is not IAsyncOperation<T>. Type: $($AsyncOp.GetType().Name)"
        return $null
    }
    
    $T = $interface.GetGenericArguments()[0]
    
    # Generic invocation
    $genericMethod = $method.MakeGenericMethod($T)
    $task = $genericMethod.Invoke($null, @($AsyncOp))
    
    $task.Wait()
    return $task.Result
}

# UUIDs
$ServiceUuid = [Guid]"0000ada5-0000-1000-8000-00805f9b34fb"
$AlertUuid   = [Guid]"0000a1e7-0000-1000-8000-00805f9b34fb"

try {
    # Create Service Provider
    Write-Host "Creating GATT Service..."
    $op = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattServiceProvider]::CreateAsync($ServiceUuid)
    $providerResult = Await($op)
    
    if ($providerResult.Error -ne "Success") {
        Write-Error "Failed to create service: $($providerResult.Error)"
        exit
    }
    $provider = $providerResult.ServiceProvider
    $service = $provider.Service

    # Create Characteristic
    Write-Host "Creating Characteristic..."
    $charParams = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattLocalCharacteristicParameters]::new()
    $charParams.CharacteristicProperties = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicProperties]::Notify
    $charParams.UserDescription = "AlertStream"
    
    $op = $service.CreateCharacteristicAsync($AlertUuid, $charParams)
    $charResult = Await($op)
    
    if ($charResult.Error -ne "Success") {
        Write-Error "Failed to create char: $($charResult.Error)"
        exit
    }
    $characteristic = $charResult.Characteristic
    Write-Host "Characteristic created."

    # Start Advertising
    Write-Host "Starting Advertising..."
    $advParams = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattServiceProviderAdvertisingParameters]::new()
    $advParams.IsConnectable = $true
    $advParams.IsDiscoverable = $true
    
    $provider.StartAdvertising($advParams)
    Write-Host "Advertising as 'Windows ADAS Fake'. Waiting for connections..."

    # Payload
    # Header: 39 30 00 00 (Tick 12345) code 12345 = 0x3039 (LE)
    # CBOR: a561...
    $hex = "39300000a561741930396176183c616801616200616181a3626964006173026172684643572054657374"
    $bytes = [byte[]]::new($hex.Length / 2)
    for($i=0; $i -lt $hex.Length; $i+=2) {
        $bytes[$i/2] = [Convert]::ToByte($hex.Substring($i, 2), 16)
    }
    
    # Create Buffer
    $buffer = [Windows.Security.Cryptography.CryptographicBuffer]::CreateFromByteArray($bytes)

    # Loop
    while ($true) {
        Start-Sleep -Seconds 3
        
        Write-Host "Sending notification... " -NoNewline
        try {
            $op = $characteristic.NotifyValueAsync($buffer)
            # NotifyValueAsync returns IAsyncOperation<GattCommunicationStatus>
            $status = Await($op)
            Write-Host "Sent ($status)."
        } catch {
            Write-Host "Error sending: $_"
        }
    }

} catch {
    Write-Error "Script Failed: $_"
}
