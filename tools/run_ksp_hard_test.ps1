param(
    [string]$KspPath = 'C:\Projects\Kerbal Space Program',
    [string]$SaveName = 'abc',
    [Parameter(Mandatory = $true)]
    [string]$RunLabel,
    [Parameter(Mandatory = $true)]
    [string]$ArchiveDirectory,
    [switch]$SkipInstall,
    [switch]$ContinueAfterTerminalFailure
)

$ErrorActionPreference = 'Stop'

if ($RunLabel -notmatch '^[A-Za-z0-9._-]+$') {
    throw 'RunLabel may contain only letters, digits, dot, underscore, and hyphen.'
}

$project = Split-Path -Parent $PSScriptRoot
$kspExecutable = Join-Path $KspPath 'KSP_x64.exe'
$logPath = Join-Path $KspPath 'KSP.log'
$telemetryPath = Join-Path $KspPath 'Ships\Script\cz10b\telemetry.csv'
$pluginData = Join-Path $KspPath 'GameData\CZ10BRecovery\PluginData'
$markerPath = Join-Path $pluginData 'launch-sea-mission-test.once'
$guardScript = Join-Path $PSScriptRoot 'watch_ksp_hard_constraints.ps1'

if (-not (Test-Path -LiteralPath $kspExecutable)) {
    throw "KSP_x64.exe not found under $KspPath"
}
if (Get-Process -Name 'KSP_x64' -ErrorAction SilentlyContinue) {
    throw 'KSP_x64 is already running. Refusing to start an ambiguous test.'
}

New-Item -ItemType Directory -Force -Path $ArchiveDirectory | Out-Null
$resultPath = Join-Path $ArchiveDirectory ($RunLabel + '-guard.txt')
$pidPath = Join-Path $ArchiveDirectory ($RunLabel + '-pid.txt')
$archiveTelemetry = Join-Path $ArchiveDirectory ($RunLabel + '-telemetry.csv')
$archiveLog = Join-Path $ArchiveDirectory ($RunLabel + '-KSP.log')

if (-not $SkipInstall) {
    & (Join-Path $PSScriptRoot 'install.ps1') `
        -KspPath $KspPath -SaveName $SaveName
}

# kOS volume 0 maps to Ships/Script.  Clearing the parent-level legacy path
# does not isolate a run and caused Run 90 to contain both Run 89 and Run 90.
if (Test-Path -LiteralPath $telemetryPath) {
    Remove-Item -LiteralPath $telemetryPath -Force
}
Remove-Item -LiteralPath $resultPath,$pidPath,$archiveTelemetry,$archiveLog `
    -Force -ErrorAction SilentlyContinue

New-Item -ItemType Directory -Force -Path $pluginData | Out-Null
Set-Content -LiteralPath $markerPath -Value $SaveName -Encoding ASCII -NoNewline

# KSP normally truncates KSP.log during launch.  Starting from the old length
# prevents a stale previous failure from killing the new process; the watcher
# resets to zero as soon as it observes truncation.
$startOffset = 0
if (Test-Path -LiteralPath $logPath) {
    $startOffset = (Get-Item -LiteralPath $logPath).Length
}

$kspProcess = Start-Process -FilePath $kspExecutable `
    -WorkingDirectory $KspPath -WindowStyle Hidden -PassThru
Set-Content -LiteralPath $pidPath -Value $kspProcess.Id -Encoding ASCII

$guardExit = 3
try {
    # Run the guard as a foreground child process.  Array splatting preserves
    # paths with spaces and avoids the detached ArgumentList quoting failure
    # that left Run 90 unguarded after the interactive wait was interrupted.
    $guardArguments = @(
        '-NoProfile',
        '-ExecutionPolicy', 'Bypass',
        '-File', $guardScript,
        '-KspProcessId', [string]$kspProcess.Id,
        '-LogPath', $logPath,
        '-StartOffset', [string]$startOffset,
        '-ResultPath', $resultPath
    )
    if ($ContinueAfterTerminalFailure) {
        $guardArguments += '-ContinueAfterTerminalFailure'
    }
    & powershell.exe @guardArguments
    $guardExit = $LASTEXITCODE
} finally {
    if (Test-Path -LiteralPath $telemetryPath) {
        Copy-Item -LiteralPath $telemetryPath `
            -Destination $archiveTelemetry -Force
    }
    if (Test-Path -LiteralPath $logPath) {
        Copy-Item -LiteralPath $logPath -Destination $archiveLog -Force
    }
    # An interrupted wrapper must fail closed.  Never leave a KSP mission
    # running after its foreground hard guard disappears.
    Stop-Process -Id $kspProcess.Id -Force -ErrorAction SilentlyContinue
}

if (-not (Test-Path -LiteralPath $archiveTelemetry)) {
    throw "No telemetry was produced for $RunLabel"
}
if ($guardExit -ne 0) {
    exit $guardExit
}
