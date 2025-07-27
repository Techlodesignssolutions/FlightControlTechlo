param(
    [int]$Port = 5000,
    [int]$MaxPackets = 10
)

Write-Host "=== FlightGear UDP Debug Tool (PowerShell) ===" -ForegroundColor Green
Write-Host "Listening on port $Port for FlightGear data..." -ForegroundColor Yellow
Write-Host "Will capture $MaxPackets packets then exit" -ForegroundColor Yellow
Write-Host "Press Ctrl+C to exit early" -ForegroundColor Yellow
Write-Host ""

try {
    # Create UDP client
    $udpClient = New-Object System.Net.Sockets.UdpClient($Port)
    $remoteEndPoint = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
    
    Write-Host "Socket bound successfully. Waiting for data..." -ForegroundColor Green
    Write-Host ""
    
    $packetCount = 0
    
    while ($packetCount -lt $MaxPackets) {
        try {
            # Set a timeout of 5 seconds
            $udpClient.Client.ReceiveTimeout = 5000
            
            # Receive data
            $receivedBytes = $udpClient.Receive([ref]$remoteEndPoint)
            $receivedString = [System.Text.Encoding]::ASCII.GetString($receivedBytes)
            
            $packetCount++
            
            Write-Host "=== Packet #$packetCount (length: $($receivedBytes.Length)) ===" -ForegroundColor Cyan
            Write-Host "From: $($remoteEndPoint.Address):$($remoteEndPoint.Port)" -ForegroundColor Gray
            
            # Print raw data with special characters visible
            Write-Host "Raw data: '" -NoNewline -ForegroundColor White
            foreach ($char in $receivedString.ToCharArray()) {
                switch ([int]$char) {
                    9  { Write-Host "[TAB]" -NoNewline -ForegroundColor Red }
                    10 { Write-Host "[LF]" -NoNewline -ForegroundColor Red }
                    13 { Write-Host "[CR]" -NoNewline -ForegroundColor Red }
                    32 { Write-Host "[SPACE]" -NoNewline -ForegroundColor Yellow }
                    default {
                        if ([int]$char -lt 32 -or [int]$char -gt 126) {
                            Write-Host "[0x$([int]$char.ToString('X2'))]" -NoNewline -ForegroundColor Magenta
                        } else {
                            Write-Host $char -NoNewline -ForegroundColor White
                        }
                    }
                }
            }
            Write-Host "'" -ForegroundColor White
            
            # Print hex dump
            Write-Host "Hex dump: " -NoNewline -ForegroundColor Gray
            foreach ($byte in $receivedBytes[0..([Math]::Min(49, $receivedBytes.Length-1))]) {
                Write-Host ("{0:X2} " -f $byte) -NoNewline -ForegroundColor Gray
            }
            Write-Host ""
            
            # Count separators
            $tabCount = ($receivedString.ToCharArray() | Where-Object { $_ -eq "`t" }).Count
            $spaceCount = ($receivedString.ToCharArray() | Where-Object { $_ -eq " " }).Count
            $commaCount = ($receivedString.ToCharArray() | Where-Object { $_ -eq "," }).Count
            
            Write-Host "Separators: $tabCount tabs, $spaceCount spaces, $commaCount commas" -ForegroundColor Yellow
            
            # Try parsing as numbers
            if ($tabCount -gt 0) {
                Write-Host "Tab-separated values:" -ForegroundColor Green
                $fields = $receivedString.Split("`t`n`r", [System.StringSplitOptions]::RemoveEmptyEntries)
                for ($i = 0; $i -lt [Math]::Min(12, $fields.Length); $i++) {
                    $field = $fields[$i].Trim()
                    $value = 0.0
                    if ([double]::TryParse($field, [ref]$value)) {
                        Write-Host "  Field $($i+1): $($value.ToString('F6'))" -ForegroundColor Green
                    } else {
                        Write-Host "  Field $($i+1): '$field' (not a number)" -ForegroundColor Red
                    }
                }
                
                # Check if this looks like the expected 9-field format
                if ($fields.Length -eq 9) {
                    Write-Host "SUCCESS: Received 9 fields (full attitude + IMU format)" -ForegroundColor Green -BackgroundColor DarkGreen
                } elseif ($fields.Length -eq 6) {
                    Write-Host "PARTIAL: Received 6 fields (IMU-only format)" -ForegroundColor Yellow -BackgroundColor DarkYellow
                } else {
                    Write-Host "WARNING: Received $($fields.Length) fields (unexpected format)" -ForegroundColor Red -BackgroundColor DarkRed
                }
            } elseif ($spaceCount -gt 0) {
                Write-Host "Space-separated values:" -ForegroundColor Green
                $fields = $receivedString.Split(" `n`r", [System.StringSplitOptions]::RemoveEmptyEntries)
                for ($i = 0; $i -lt [Math]::Min(12, $fields.Length); $i++) {
                    $field = $fields[$i].Trim()
                    $value = 0.0
                    if ([double]::TryParse($field, [ref]$value)) {
                        Write-Host "  Field $($i+1): $($value.ToString('F6'))" -ForegroundColor Green
                    } else {
                        Write-Host "  Field $($i+1): '$field' (not a number)" -ForegroundColor Red
                    }
                }
            }
            
            Write-Host ""
            
        } catch [System.Net.Sockets.SocketException] {
            if ($_.Exception.SocketErrorCode -eq [System.Net.Sockets.SocketError]::TimedOut) {
                Write-Host "No data received in 5 seconds. Is FlightGear running and configured correctly?" -ForegroundColor Red
                Write-Host "Expected FlightGear command:" -ForegroundColor Yellow
                Write-Host "fgfs --generic=socket,out,10,127.0.0.1,$Port,udp,output_protocol" -ForegroundColor Cyan
                break
            } else {
                throw
            }
        }
    }
    
    if ($packetCount -gt 0) {
        Write-Host "Captured $packetCount packets successfully!" -ForegroundColor Green
    }
    
} catch {
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
    if ($_.Exception.InnerException) {
        Write-Host "Inner: $($_.Exception.InnerException.Message)" -ForegroundColor Red
    }
} finally {
    if ($udpClient) {
        $udpClient.Close()
        Write-Host "UDP client closed." -ForegroundColor Gray
    }
}

Write-Host ""
Write-Host "Debug complete. Next steps:" -ForegroundColor Yellow
Write-Host "1. If you saw 9 fields: Your parser should work with the full format" -ForegroundColor White
Write-Host "2. If you saw 6 fields: Use the IMU-only protocol" -ForegroundColor White
Write-Host "3. If no data: Check FlightGear configuration and port numbers" -ForegroundColor White 