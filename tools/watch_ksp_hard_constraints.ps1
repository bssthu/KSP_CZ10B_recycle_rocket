param(
    [Parameter(Mandatory = $true)]
    [int]$KspProcessId,
    [Parameter(Mandatory = $true)]
    [string]$LogPath,
    [long]$StartOffset = 0,
    [string]$ResultPath = '',
    [switch]$ContinueAfterTerminalFailure
)

$ErrorActionPreference = 'Stop'
$offset = $StartOffset
$pending = ''
$script:terminalFailure = ''
$script:terminalNominalMiss = ''

function Write-GuardResult {
    param([string]$Message)
    if ($ResultPath) {
        Set-Content -LiteralPath $ResultPath -Value $Message -Encoding UTF8
    }
}

function Stop-FailedRun {
    param([string]$Reason)
    if ($script:terminalFailure) {
        $Reason = $script:terminalFailure + '; downstream=' + $Reason
    }
    Write-GuardResult ("FAIL " + (Get-Date -Format o) + " " + $Reason)
    Stop-Process -Id $KspProcessId -Force -ErrorAction SilentlyContinue
    exit 2
}

function Stop-PassedRun {
    param([string]$Evidence)
    if ($script:terminalFailure) {
        Stop-FailedRun ("later in-process pass was invalid after formal failure: " +
            $Evidence)
    }
    $tier = if ($script:terminalNominalMiss) { 'RECOVERED' } else { 'NOMINAL' }
    $nominalEvidence = if ($script:terminalNominalMiss) {
        '; nominalMiss=' + $script:terminalNominalMiss
    } else {
        ''
    }
    Write-GuardResult (
        "PASS " + (Get-Date -Format o) + " tier=$tier " +
        $Evidence + $nominalEvidence)
    Stop-Process -Id $KspProcessId -Force -ErrorAction SilentlyContinue
    exit 0
}

Write-GuardResult ("WATCHING " + (Get-Date -Format o) + " pid=$KspProcessId offset=$offset")

while (Get-Process -Id $KspProcessId -ErrorAction SilentlyContinue) {
    if (Test-Path -LiteralPath $LogPath) {
        $length = (Get-Item -LiteralPath $LogPath).Length
        if ($length -lt $offset) {
            $offset = 0
            $pending = ''
        }
        if ($length -gt $offset) {
            $stream = [System.IO.File]::Open(
                $LogPath,
                [System.IO.FileMode]::Open,
                [System.IO.FileAccess]::Read,
                [System.IO.FileShare]::ReadWrite
            )
            try {
                [void]$stream.Seek($offset, [System.IO.SeekOrigin]::Begin)
                $reader = [System.IO.StreamReader]::new($stream)
                try {
                    $chunk = $reader.ReadToEnd()
                    $offset = $stream.Position
                } finally {
                    $reader.Dispose()
                }
            } finally {
                $stream.Dispose()
            }

            $text = $pending + $chunk
            $lines = $text -split "`r?`n"
            if ($text -notmatch "`r?`n$") {
                $pending = $lines[-1]
                if ($lines.Count -gt 1) {
                    $lines = $lines[0..($lines.Count - 2)]
                } else {
                    $lines = @()
                }
            } else {
                $pending = ''
            }

            foreach ($line in $lines) {
                if ($line -match 'TERMINAL_2KM verticalVelocity=([-0-9.]+) descent=([-0-9.]+) horizontal=([-0-9.]+) hookError=([-0-9.]+) nozzleAngleMax=([-0-9.]+)') {
                    $descent = [double]$matches[2]
                    $horizontal = [double]$matches[3]
                    $hookError = [double]$matches[4]
                    $nozzle = [double]$matches[5]
                    $terminalEvidence = "TERMINAL_2KM descent=$descent horizontal=$horizontal hookError=$hookError nozzle=$nozzle"
                    $commonAccepted = $descent -ge 150.0 -and
                        $descent -le 200.0 -and $nozzle -le 30.0
                    $nominalAccepted = $commonAccepted -and
                        $horizontal -le 5.0 -and $hookError -le 10.0
                    $recoveryAccepted = $commonAccepted -and
                        $horizontal -le 10.0 -and $hookError -le 30.0
                    if (-not $recoveryAccepted) {
                        if ($ContinueAfterTerminalFailure) {
                            $script:terminalFailure = $terminalEvidence
                            Write-GuardResult (
                                "TERMINAL_FAIL_CONTINUING " +
                                (Get-Date -Format o) + " " +
                                $terminalEvidence)
                        } else {
                            Stop-FailedRun $terminalEvidence
                        }
                    } elseif (-not $nominalAccepted) {
                        $script:terminalNominalMiss = $terminalEvidence
                        Write-GuardResult (
                            "TERMINAL_RECOVERY_ADMITTED " +
                            (Get-Date -Format o) + " " +
                            $terminalEvidence)
                    } else {
                        Write-GuardResult (
                            "TERMINAL_NOMINAL_PASS " + $terminalEvidence)
                    }
                }
                if ($line -match 'SEA_MISSION_TEST_STATUS' -and
                    $line -match 'nozzleViolation=True') {
                    Stop-FailedRun 'powered nozzle/velocity angle exceeded 30 degrees'
                }
                if ($line -match 'MAIN_BURN_THROTTLE_VIOLATION') {
                    Stop-FailedRun 'main burn throttle left the mandatory 75--100 percent corridor above 2 km'
                }
                if ($line -match 'BOOSTER_WATER_CONTACT' -or
                    ($line -match 'SEA_MISSION_TEST_STATUS' -and
                     $line -match 'situation=SPLASHED')) {
                    Stop-FailedRun 'booster water contact'
                }
                if ($line -match '\[CZ10BNetRecovery\] CONSTRAINT_FAIL G-05N_2KM_NOMINAL') {
                    if (-not $script:terminalNominalMiss) {
                        $script:terminalNominalMiss = $line.Trim()
                    }
                    continue
                }
                if ($line -match '\[CZ10BNetRecovery\] CONSTRAINT_FAIL ') {
                    Stop-FailedRun $line.Trim()
                }
                if ($line -match '\[CZ10BNetRecovery\] (?:SEA_)?MISSION_TEST_(?:RELEASE_FAILED|FAIL)') {
                    Stop-FailedRun $line.Trim()
                }
                if ($line -match '\[CZ10BNetRecovery\] SEA_MISSION_TEST_PASS') {
                    Stop-PassedRun 'complete in-process acceptance matrix passed after capture dwell'
                }
            }
        }
    }
    Start-Sleep -Milliseconds 200
}

if ($script:terminalFailure) {
    Write-GuardResult (
        "FAIL " + (Get-Date -Format o) + " " +
        $script:terminalFailure + "; downstream=process exited")
    exit 2
}
Write-GuardResult ("PROCESS_EXIT " + (Get-Date -Format o))
exit 3
