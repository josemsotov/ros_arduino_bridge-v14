$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing

$size = 240
$resolution = 0.05
$origin = -6.0
$innerWall = 4.9
$output = Join-Path (Split-Path -Parent $PSScriptRoot) `
    'src\follower_sim\maps\follower_world.png'

$bitmap = [System.Drawing.Bitmap]::new($size, $size)
try {
    for ($pixelY = 0; $pixelY -lt $size; $pixelY++) {
        $worldY = $origin + (($size - 1 - $pixelY) + 0.5) * $resolution
        for ($pixelX = 0; $pixelX -lt $size; $pixelX++) {
            $worldX = $origin + ($pixelX + 0.5) * $resolution
            $free = ([math]::Abs($worldX) -lt $innerWall) -and `
                    ([math]::Abs($worldY) -lt $innerWall)
            $value = if ($free) { 254 } else { 0 }
            $bitmap.SetPixel(
                $pixelX, $pixelY,
                [System.Drawing.Color]::FromArgb($value, $value, $value))
        }
    }
    $bitmap.Save($output, [System.Drawing.Imaging.ImageFormat]::Png)
}
finally {
    $bitmap.Dispose()
}

Write-Host "Generated $output ($size x $size at $resolution m/pixel)"
