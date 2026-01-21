try {
    $type = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattServiceProvider, Windows.Devices.Bluetooth, ContentType=WindowsRuntime]
    Write-Host "Success: $($type.FullName)"
} catch {
    Write-Host "Error: $_"
}
