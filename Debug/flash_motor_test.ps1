# =====================================================================
# flash_motor_test.ps1 - one-click build + JLink flash
#   Target: motor_test robot / GIMBAL_BOARD (STM32F407IG)
#
# Usage (VS Code task "one-click flash" or terminal):
#   powershell -NoProfile -ExecutionPolicy Bypass -File Debug\flash_motor_test.ps1
# =====================================================================

$ErrorActionPreference = "Stop"

$RobotType = "motor_test"
$BoardType = "GIMBAL_BOARD"

# project root = parent of this script's folder (Debug\)
$Root  = Split-Path -Parent $PSScriptRoot
$Build = Join-Path $Root "cmake-build-$RobotType"
$Hex   = Join-Path $Build "control-2026.hex"

# target device by board type
if ($BoardType -eq "GIMBAL_BOARD") { $Device = "STM32F407IG" }
else                                { $Device = "STM32H723ZG" }

Write-Host "==> Configure ($RobotType / $BoardType)" -ForegroundColor Cyan
cmake -G Ninja -S "$Root" -B "$Build" -DCMAKE_BUILD_TYPE=Debug "-DROBOT_TYPE=$RobotType" "-DBOARD_TYPE=$BoardType"
if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed"; exit 1 }

Write-Host "==> Build" -ForegroundColor Cyan
cmake --build "$Build"
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; exit 1 }
if (-not (Test-Path -LiteralPath $Hex)) { Write-Error "hex not found: $Hex"; exit 1 }

# locate JLink.exe
$JLink = $null
$fromPath = Get-Command JLink.exe -ErrorAction SilentlyContinue
if ($fromPath) { $JLink = $fromPath.Source }
if (-not $JLink) {
    $cand = Get-ChildItem -Path "C:\Program Files\SEGGER","C:\Program Files (x86)\SEGGER" -Filter "JLink.exe" -Recurse -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending | Select-Object -First 1
    if ($cand) { $JLink = $cand.FullName }
}
if (-not $JLink) { Write-Error "JLink.exe not found (install SEGGER J-Link or add to PATH)"; exit 1 }
Write-Host "==> JLink: $JLink" -ForegroundColor Cyan

# generate JLink commander script
$Script = Join-Path $env:TEMP "flash_$RobotType.jlink"
$HexUnix = $Hex -replace '\\','/'
$lines = @(
    "si SWD",
    "speed 4000",
    "device $Device",
    "connect",
    "r",
    "h",
    "loadfile $HexUnix",
    "r",
    "g",
    "qc"
)
Set-Content -LiteralPath $Script -Value $lines -Encoding ASCII

Write-Host "==> Flash ($Device)" -ForegroundColor Cyan
& $JLink -device $Device -if SWD -speed 4000 -autoconnect 1 -CommanderScript $Script
if ($LASTEXITCODE -ne 0) { Write-Error "Flash failed"; exit 1 }

Write-Host "==> Done!" -ForegroundColor Green
