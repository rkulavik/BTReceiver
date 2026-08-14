# Lightweight HTTP Server in PowerShell using .NET HttpListener
# Provides a Secure Context (http://localhost:8080) required by Web Bluetooth

param (
    [int]$Port = 8080,
    [switch]$NoBrowser
)

$prefix = "http://localhost:$Port/"
$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add($prefix)

try {
    $listener.Start()
    Write-Host "=========================================================================" -ForegroundColor Green
    Write-Host "  Ranchbot BLE Receiver Server Running at: http://localhost:$Port/" -ForegroundColor Cyan
    Write-Host "  Web Bluetooth Secure Context Active. Browser can access hardware BT radio!" -ForegroundColor Yellow
    Write-Host "=========================================================================" -ForegroundColor Green

    # Launch default browser if requested
    if (-not $NoBrowser) {
        Start-Process $prefix
    }

    while ($listener.IsListening) {
        $context = $listener.GetContext()
        $request = $context.Request
        $response = $context.Response

        $path = $request.Url.LocalPath
        if ($path -eq "/") { $path = "/index.html" }

        $localPath = Join-Path (Get-Location) $path.TrimStart('/')

        if (Test-Path $localPath -PathType Leaf) {
            $bytes = [System.IO.File]::ReadAllBytes($localPath)
            
            # Content Type Mapping
            $ext = [System.IO.Path]::GetExtension($localPath).ToLower()
            switch ($ext) {
                ".html" { $response.ContentType = "text/html; charset=utf-8" }
                ".css"  { $response.ContentType = "text/css; charset=utf-8" }
                ".js"   { $response.ContentType = "application/javascript; charset=utf-8" }
                ".json" { $response.ContentType = "application/json; charset=utf-8" }
                ".png"  { $response.ContentType = "image/png" }
                default { $response.ContentType = "application/octet-stream" }
            }

            $response.ContentLength64 = $bytes.Length
            $response.OutputStream.Write($bytes, 0, $bytes.Length)
        } else {
            $response.StatusCode = 404
            $notFound = [System.Text.Encoding]::UTF8.GetBytes("404 Not Found")
            $response.OutputStream.Write($notFound, 0, $notFound.Length)
        }
        $response.Close()
    }
} catch {
    Write-Host "Error starting server: $_" -ForegroundColor Red
} finally {
    $listener.Stop()
}
