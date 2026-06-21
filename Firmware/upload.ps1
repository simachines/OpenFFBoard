param(
    [ValidateSet('F407VG', 'F407VG_DISCO', 'F411RE')]
    [string]$Target = 'F407VG'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

. "$PSScriptRoot\_toolchain_windows.ps1"
Initialize-FFBToolchain -NeedProgrammer

Push-Location $PSScriptRoot
try {
    & make "MCU_TARGET=$Target" flash
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}

