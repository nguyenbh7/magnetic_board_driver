# Package a compiled Windows live fitting app with its own DICOM source and
# lab-laptop installer. Build on Windows; no experimental data is bundled.
# From repo root:
#   powershell -ExecutionPolicy Bypass -File .\scripts\package_lab_laptop.ps1
# Use -SkipBuild if target/release/app.exe is already built and no app is running.
param(
    [switch]$SkipBuild,
    [string]$OutputZip = ""
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$binary = Join-Path $repo "target\release\app.exe"
$pythonSource = Join-Path $repo "app\python_cbct"
$setup = Join-Path $repo "scripts\setup_dicom_windows.ps1"

if (-not $SkipBuild) {
    Write-Host "Building release app (close any running app.exe first)..."
    Push-Location $repo
    try {
        & cargo build -p app --release
        if ($LASTEXITCODE -ne 0) {
            throw "Rust build failed. Close all app.exe windows if Cargo reports Access is denied."
        }
    } finally { Pop-Location }
}
foreach ($file in @($binary, (Join-Path $pythonSource "requirements.txt"),
                     (Join-Path $pythonSource "check_dicom_environment.py"), $setup)) {
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) {
        throw "Package incomplete, file not found: $file"
    }
}

if ([string]::IsNullOrWhiteSpace($OutputZip)) {
    $OutputZip=Join-Path $repo "dist\GK_Live_Fitting_Lab_Laptop.zip"
}
$OutputZip=[System.IO.Path]::GetFullPath($OutputZip)
if (Test-Path -LiteralPath $OutputZip) {
    throw "Refusing to overwrite existing lab package: $OutputZip. Choose a NEW -OutputZip."
}
$destination=[System.IO.Path]::GetDirectoryName($OutputZip)
[void](New-Item -ItemType Directory -Path $destination -Force)
$temp=Join-Path ([System.IO.Path]::GetTempPath()) ("gk_lab_laptop_" + [guid]::NewGuid().ToString("N"))
$folder=Join-Path $temp "GK_Live_Fitting_Lab_Laptop"
try {
    $targetPython=Join-Path $folder "app\python_cbct"
    $targetScripts=Join-Path $folder "scripts"
    [void](New-Item -ItemType Directory -Path $targetPython -Force)
    [void](New-Item -ItemType Directory -Path $targetScripts -Force)
    Copy-Item -LiteralPath $binary -Destination (Join-Path $folder "app.exe")
    Get-ChildItem -LiteralPath $pythonSource -File |
        Where-Object { $_.Extension -eq ".py" -or $_.Name -in @("requirements.txt", "README.md") } |
        Copy-Item -Destination $targetPython
    Copy-Item -LiteralPath $setup -Destination (Join-Path $targetScripts "setup_dicom_windows.ps1")

    $readme = @'
GK live fitting app — lab laptop quick start

1) Extract the WHOLE ZIP to a directory on the laptop.
2) Install Python 3.11 and the Windows Python launcher "py" if absent.
   Internet access is needed for the one-time package installation.
3) Open PowerShell in THIS extracted folder. Run:
       powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1
4) Start .\app.exe. Open DICOM Geometry -> Check DICOM packages.
   If a Python environment error occurs, click Copy setup commands.
5) Original / Optimized use the packaged Python detector. Hybrid is NOT
   available until the separately reviewed algorithm is integrated.
6) If you previously set CBCT_DETECT_SCRIPT or CBCT_PYTHON to another
   installation, remove those overrides so the app picks the bundled code.
7) Keep app.exe and app/python_cbct together. No other Git checkout needed.

The DICOM importer uses Python + the scientific packages in
app/python_cbct/requirements.txt. Hall streaming and Rust magnetic fitting
do NOT require those Python packages. The app does not install anything
automatically. This is experimental GK tracking software; no clinical use.
'@
    Set-Content -LiteralPath (Join-Path $folder "README_LAB_LAPTOP.txt") -Value $readme -Encoding UTF8
    Compress-Archive -LiteralPath $folder -DestinationPath $OutputZip -CompressionLevel Optimal
    Write-Host "Created portable app + offline source ZIP: $OutputZip"
    Write-Host "On the lab laptop: extract, run scripts\setup_dicom_windows.ps1, then app.exe"
    Write-Host "Python packages download once during setup; they are not bundled in this ZIP."
} finally {
    if (Test-Path -LiteralPath $temp) {
        Remove-Item -LiteralPath $temp -Recurse -Force
    }
}
