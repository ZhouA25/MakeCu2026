# One-shot setup of a Windows computer to run PillBot, using a plain venv (.venv). Safe to re-run.
# Windows version of setup_laptop.sh. Run in PowerShell from the repo root:
#   Set-ExecutionPolicy -Scope Process Bypass        # once per terminal, lets this script run
#   .\scripts\setup_laptop.ps1
#   .\scripts\setup_laptop.ps1 -Bundle pillbot-local-....tgz   # + import the old computer's local files
#   .\scripts\setup_laptop.ps1 -NoSim                         # skip the simulator packages
#   .\scripts\setup_laptop.ps1 -NoHub                         # no ESP32 hub: hub.enabled false (no clamp/buttons)
#
# LeRobot 0.7 pins numpy<2.3, which has no Python 3.13+/3.14 wheels on Windows, so .venv must be
# Python 3.12. An existing .venv on another version is renamed to .venv-py<ver>-old, not deleted.
param([string]$Bundle = "", [switch]$NoSim, [switch]$NoHub)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path $PSScriptRoot -Parent)
$env:PYTHONUTF8 = "1"   # status output uses non-ASCII characters; avoid cp1252 errors
$Venv = ".venv"
$VPy = "$Venv\Scripts\python.exe"

function Run([string]$what, [scriptblock]$cmd, [switch]$optional) {
    & $cmd
    if ($LASTEXITCODE -ne 0) {
        if ($optional) { Write-Host "   ($what failed - skipped)"; return }
        throw "$what failed (exit $LASTEXITCODE)"
    }
}

# ---- 1. a Python 3.12 venv
if (Test-Path $VPy) {
    $ver = & $VPy -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')"
    if ($ver -ne "3.12") {
        $old = ".venv-py$($ver -replace '\.','')-old"
        if (Test-Path $old) { $old = "$old-$(Get-Date -Format yyyyMMddHHmmss)" }
        Write-Host "== existing .venv is Python $ver (LeRobot needs 3.12): keeping it as $old"
        Rename-Item $Venv $old
    }
}
if (-not (Test-Path $VPy)) {
    $base = $null
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $ErrorActionPreference = "Continue"   # py prints to stderr when 3.12 is missing
        try { & py -3.12 -c "pass" 2>$null; if ($LASTEXITCODE -eq 0) { $base = @("py", "-3.12") } } catch { }
        $ErrorActionPreference = "Stop"
    }
    if (-not $base) {
        throw @"
Python 3.12 not found. Install it, then re-run this script:
    winget install -e --id Python.Python.3.12
(or the 3.12 installer from python.org; tick 'py launcher'). Your other Pythons are untouched.
"@
    }
    Write-Host "== creating .venv with Python 3.12"
    Run "venv" { & $base[0] $base[1] -m venv $Venv }
}
Run "pip upgrade" { & $VPy -m pip install -q --upgrade pip }

# ---- 2. code + drivers
Write-Host "== git submodules (SensX driver)"
Run "submodule update" { git submodule update --init --recursive } -optional

Write-Host "== installing pillbot + LeRobot 0.7 + our arm plugin (a few minutes: torch is large)"
$extras = if ($NoSim) { "dev" } else { "dev,sim" }
Run "pip install pillbot" { & $VPy -m pip install -q -e ".[$extras]" }
Run "pip install arm plugin" { & $VPy -m pip install -q -e plugins/lerobot_robot_pillbot }
if ((Test-Path sensx_python/pyproject.toml) -or (Test-Path sensx_python/setup.py)) {
    Run "pip install sensx" { & $VPy -m pip install -q -e sensx_python } -optional
}
if (-not $NoSim) {   # used by the assembly checks and validation reports
    Run "pip install sim tools" { & $VPy -m pip install -q trimesh scipy matplotlib } -optional
}

# ---- 3. local config
if ($Bundle) {
    Write-Host "== importing local files from $Bundle"
    Run "import" { & $VPy -m pillbot.tools.setup_local import $Bundle }
}
if ($NoHub) {
    Write-Host "== detecting the U2D2 + camera -> configs/robot.local.yaml (no hub: plug in the U2D2 first)"
    Run "init" { & $VPy -m pillbot.tools.setup_local init --no-hub }
} else {
    Write-Host "== detecting ports + camera -> configs/robot.local.yaml (plug in the U2D2 and the hub first)"
    Run "init" { & $VPy -m pillbot.tools.setup_local init }
}
Write-Host ""
& $VPy -m pillbot.tools.setup_local check
Write-Host ""
if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) {
    Write-Host "Note: no ffmpeg on PATH. Scripted runs don't need it; LeRobot dataset recording does:"
    Write-Host "      winget install -e --id Gyan.FFmpeg.Shared"
}
Write-Host "Windows: serial ports are COM numbers (Device Manager > Ports). If Windows Firewall asks when"
Write-Host "the camera or live viewer first starts, allow Python on Private networks."
Write-Host ""
if ($NoHub) {
    Write-Host "No hub: no bottle clamp and no STOP button. Hold the bottle another way, and stop a run"
    Write-Host "with Ctrl-C (keep a hand on the arm's power switch). Don't use --trigger button/zone."
    Write-Host ""
}
Write-Host "Done. Each new PowerShell:  .\.venv\Scripts\Activate.ps1"
Write-Host "Git Bash (for scripts/*.sh):  source .venv/Scripts/activate"
