Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Add-PathOnce {
    param(
        [Parameter(Mandatory = $true)][string]$PathToAdd
    )

    if (-not (Test-Path $PathToAdd)) {
        return
    }

    $parts = $env:PATH -split ';'
    if ($parts -notcontains $PathToAdd) {
        $env:PATH = "$PathToAdd;$env:PATH"
    }
}

function Find-LatestBinaryDir {
    param(
        [Parameter(Mandatory = $true)][string]$Root,
        [Parameter(Mandatory = $true)][string]$FileName
    )

    if (-not (Test-Path $Root)) {
        return $null
    }

    return (
        Get-ChildItem -Path $Root -Filter $FileName -Recurse -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1 -ExpandProperty DirectoryName
    )
}

function Initialize-FFBToolchain {
    param(
        [switch]$NeedProgrammer
    )

    $toolRoots = @()
    if ($env:STM32_TOOLCHAIN_ROOT) { $toolRoots += $env:STM32_TOOLCHAIN_ROOT }
    $toolRoots += 'C:\ST'

    $makeBin = $null
    $gccBin = $null

    foreach ($root in $toolRoots) {
        if (-not $makeBin) {
            $makeBin = Find-LatestBinaryDir -Root $root -FileName 'make.exe'
        }
        if (-not $gccBin) {
            $gccBin = Find-LatestBinaryDir -Root $root -FileName 'arm-none-eabi-gcc.exe'
        }
    }

    if (-not $makeBin -or -not $gccBin) {
        throw 'Could not find make.exe and arm-none-eabi-gcc.exe. Install STM32CubeIDE/toolchain or set STM32_TOOLCHAIN_ROOT.'
    }

    Add-PathOnce -PathToAdd $makeBin
    Add-PathOnce -PathToAdd $gccBin

    if ($NeedProgrammer) {
        $programmer = Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue
        if (-not $programmer) {
            $programmerCandidates = @(
                'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin',
                'C:\ST\STM32CubeProgrammer\bin'
            )

            foreach ($dir in $programmerCandidates) {
                if (Test-Path (Join-Path $dir 'STM32_Programmer_CLI.exe')) {
                    Add-PathOnce -PathToAdd $dir
                    break
                }
            }
        }

        if (-not (Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue)) {
            throw 'STM32_Programmer_CLI not found. Install STM32CubeProgrammer and ensure it is available in PATH.'
        }
    }
}