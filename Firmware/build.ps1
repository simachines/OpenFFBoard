param(
    [ValidateSet('F407VG', 'F407VG_DISCO', 'F411RE')]
    [string]$Target = 'F407VG',
    [int]$Jobs = 8,
    [switch]$Release
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

. "$PSScriptRoot\_toolchain_windows.ps1"
Initialize-FFBToolchain

$debugFlag = if ($Release) { '0' } else { '1' }

Push-Location $PSScriptRoot
try {
    & make "MCU_TARGET=$Target" "DEBUG=$debugFlag" "-j$Jobs"
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}