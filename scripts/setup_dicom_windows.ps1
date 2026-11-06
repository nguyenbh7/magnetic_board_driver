# One-time DICOM Python setup for the Gamma Knife live fitting app (Windows).
# No administrator privileges, no global Python package modifications.
# Run from any directory:
#   powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1
# Re-check later (no installations):
#   powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1 -CheckOnly
param([switch]$CheckOnly)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$requirements = Join-Path $repo "app\python_cbct\requirements.txt"
$checker = Join-Path $repo "app\python_cbct\check_dicom_environment.py"
$venv = Join-Path $repo ".venv"
$python = Join-Path $venv "Scripts\python.exe"

if (-not (Test-Path -LiteralPath $requirements -PathType Leaf)) {
    throw "Missing DICOM requirements: $requirements. Update the app repository."
}
if (-not (Test-Path -LiteralPath $checker -PathType Leaf)) {
    throw "Missing DICOM dependency checker: $checker. Update the app repository."
}

Write-Host "GK CBCT Python environment: $venv"
Write-Host "DICOM packages: $requirements"
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    if ($CheckOnly) {
        throw "No DICOM .venv yet. Run this script without -CheckOnly to create it."
    }
    if (-not (Get-Command py -ErrorAction SilentlyContinue)) {
        throw "Python launcher 'py' not found. Install Python 3.11 (including its Windows launcher), then rerun."
    }
    & py -3.11 -m venv $venv
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $python -PathType Leaf)) {
        throw "Couldn't create the Python 3.11 virtual environment. Install Python 3.11 and retry."
    }
}

if (-not $CheckOnly) {
    Write-Host "Installing/updating only the local DICOM virtual environment..."
    & $python -m pip install -r $requirements
    if ($LASTEXITCODE -ne 0) {
        throw "DICOM Python package installation failed. The packages were not installed globally. Review the pip error, then rerun this script."
    }
}

Write-Host "Verifying required imports and DICOM detector command..."
& $python $checker
if ($LASTEXITCODE -ne 0) {
    throw "DICOM dependencies are not ready. Read failures above; the rest of the live app can still operate independently."
}
if ($env:CBCT_DETECT_SCRIPT) {
    Write-Warning "CBCT_DETECT_SCRIPT points to '$env:CBCT_DETECT_SCRIPT'. This overrides the bundled detector. Remove this override before a lab measurement if it refers to old code."
}
if ($env:CBCT_PYTHON -and $env:CBCT_PYTHON -ne $python) {
    Write-Warning "CBCT_PYTHON points to '$env:CBCT_PYTHON'. This overrides the .venv checked above."
}
Write-Host "GK DICOM Python setup verified. Start the live app and select Original or Optimized."
Write-Host "Hybrid still needs its independently reviewed board detector implementation."
