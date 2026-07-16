[CmdletBinding()]
param(
    [string]$UpstreamDir = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$installRoot = Join-Path $env:USERPROFILE ".silabs\slt\installs"
$conanRoot = Join-Path $installRoot "conan\p"

if (-not (Test-Path -LiteralPath $conanRoot)) {
    throw "Silicon Labs tools were not found at $installRoot. Install the required SDKs with Simplicity Studio first."
}

if ([string]::IsNullOrWhiteSpace($UpstreamDir)) {
    $UpstreamDir = Join-Path (Split-Path -Parent $repoRoot) "ExpressLRS\src"
}

$resolvedUpstream = Resolve-Path -LiteralPath $UpstreamDir -ErrorAction Stop
if (-not (Test-Path -LiteralPath (Join-Path $resolvedUpstream "src\tx_main.cpp"))) {
    throw "UpstreamDir must be an ExpressLRS src directory containing src\tx_main.cpp: $resolvedUpstream"
}

$sdkPath = $null
$gccExe = $null
$ninjaExe = $null
foreach ($package in Get-ChildItem -LiteralPath $conanRoot -Directory) {
    $packageRoot = Join-Path $package.FullName "p"
    $properties = Join-Path $packageRoot ".properties"
    if ((Test-Path -LiteralPath $properties) -and $null -eq $sdkPath) {
        $contents = Get-Content -LiteralPath $properties -Raw
        if ($contents.Contains("id=com.silabs.sdk.stack.sisdk") -and
            $contents.Contains("version=2025.6.2")) {
            $sdkPath = $packageRoot
        }
    }

    $gccCandidate = Join-Path $packageRoot "bin\arm-none-eabi-gcc.exe"
    if ((Test-Path -LiteralPath $gccCandidate) -and $null -eq $gccExe) {
        $gccExe = $gccCandidate
    }

    $ninjaCandidate = Join-Path $packageRoot "ninja.exe"
    if ((Test-Path -LiteralPath $ninjaCandidate) -and $null -eq $ninjaExe) {
        $ninjaExe = $ninjaCandidate
    }
}

if ($null -eq $sdkPath) {
    throw "Simplicity SDK 2025.6.2 is not installed."
}
if ($null -eq $gccExe) {
    throw "The Silicon Labs Arm GNU toolchain was not found."
}
if ($null -eq $ninjaExe) {
    throw "The Silicon Labs Ninja executable was not found."
}

$commanderExe = Join-Path $installRoot "archive\Simplicity Commander\commander.exe"
if (-not (Test-Path -LiteralPath $commanderExe)) {
    throw "Simplicity Commander was not found at $commanderExe."
}

$cmakeCommand = Get-Command cmake -ErrorAction SilentlyContinue
if ($null -eq $cmakeCommand) {
    throw "CMake 3.25 or newer is required and was not found in PATH."
}

function Convert-ToCMakePath {
    param([Parameter(Mandatory = $true)][string]$Path)
    return $Path.Replace("\", "/")
}

$env:SILABS_SDK_PATH = Convert-ToCMakePath $sdkPath
$env:SILABS_PKG_PATH = Convert-ToCMakePath $installRoot
$env:ARM_GCC_DIR = Convert-ToCMakePath (Split-Path -Parent (Split-Path -Parent $gccExe))
$env:NINJA_EXE_PATH = Convert-ToCMakePath $ninjaExe
$env:POST_BUILD_EXE = Convert-ToCMakePath $commanderExe
$env:ELRS_UPSTREAM_DIR = Convert-ToCMakePath $resolvedUpstream.Path

Write-Host "Simplicity SDK: $env:SILABS_SDK_PATH"
Write-Host "ExpressLRS:    $env:ELRS_UPSTREAM_DIR"
Write-Host "Arm GCC:       $gccExe"
Write-Host "Ninja:         $ninjaExe"
Write-Host "Commander:     $commanderExe"

$cmakeDir = Join-Path $repoRoot "cmake_gcc"
Push-Location $cmakeDir
try {
    & $cmakeCommand.Source --workflow --preset tx-clean-port
    if ($LASTEXITCODE -ne 0) {
        throw "CMake build failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

$artifact = Join-Path $cmakeDir "build-tx-clean-port\base\wifi_gspi_merged.rps"
if (-not (Test-Path -LiteralPath $artifact)) {
    throw "Build completed without the expected RPS artifact: $artifact"
}

$hash = Get-FileHash -Algorithm SHA256 -LiteralPath $artifact
Write-Host "Firmware: $artifact"
Write-Host "SHA256:   $($hash.Hash)"
