param(
    [string]$KspPath = 'C:\Projects\Kerbal Space Program',
    [string]$SaveName = 'abc'
)

$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$gameData = Join-Path $KspPath 'GameData'
$scriptRoot = Join-Path $KspPath 'Ships\Script'
$saveVab = Join-Path $KspPath ("saves\{0}\Ships\VAB" -f $SaveName)
$installedTemplates = Join-Path $gameData 'CZ10BRecovery\CraftTemplates\VAB'

if (-not (Test-Path (Join-Path $KspPath 'KSP_x64.exe'))) {
    throw "KSP_x64.exe not found under $KspPath"
}
if (-not (Test-Path (Join-Path $gameData 'kOS\Plugins\kOS.dll'))) {
    throw 'kOS must be installed before installing the recovery demonstrator.'
}

New-Item -ItemType Directory -Force -Path $scriptRoot,(Join-Path $scriptRoot 'cz10b'),(Join-Path $scriptRoot 'boot'),$saveVab,$installedTemplates | Out-Null
Copy-Item -LiteralPath (Join-Path $project 'GameData\CZ10BRecovery') -Destination $gameData -Recurse -Force
Copy-Item -Path (Join-Path $project 'CraftTemplates\VAB\*.craft') -Destination $installedTemplates -Force
Copy-Item -Path (Join-Path $project 'Ships\Script\cz10b\*') -Destination (Join-Path $scriptRoot 'cz10b') -Force
Copy-Item -Path (Join-Path $project 'Ships\Script\boot\*.ks') -Destination (Join-Path $scriptRoot 'boot') -Force
Copy-Item -Path (Join-Path $project 'CraftTemplates\VAB\*.craft') -Destination $saveVab -Force

Write-Output "Installed CZ10BRecovery into $KspPath for save $SaveName"
