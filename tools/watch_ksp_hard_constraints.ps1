param(
    [Parameter(Mandatory = $true)]
    [int]$KspProcessId,
    [Parameter(Mandatory = $true)]
    [string]$LogPath,
    [long]$StartOffset = 0,
    [string]$ResultPath = ''
)

$ErrorActionPreference = 'Stop'
$offset = $StartOffset
$pending = ''

function Write-GuardResult {
    param([string]$Message)
    if ($ResultPath) {
        Set-Content -LiteralPath $ResultPath -Value $Message -Encoding UTF8
    }
}

function Stop-FailedRun {
    param([string]$Reason)
    Write-GuardResult ("FAIL " + (Get-Date -Format o) + " " + $Reason)
    Stop-Process -Id $KspProcessId -Force -ErrorAction SilentlyContinue
    exit 2
}

function Stop-PassedRun {
    param([string]$Evidence)
    Write-GuardResult ("PASS " + (Get-Date -Format o) + " " + $Evidence)
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
                    if ($descent -lt 150.0 -or $descent -gt 200.0 -or
                        $horizontal -gt 5.0 -or $hookError -gt 10.0 -or
                        $nozzle -gt 30.0) {
                        Stop-FailedRun "TERMINAL_2KM descent=$descent horizontal=$horizontal hookError=$hookError nozzle=$nozzle"
                    }
                    Write-GuardResult "TERMINAL_PASS descent=$descent horizontal=$horizontal hookError=$hookError nozzle=$nozzle"
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

Write-GuardResult ("PROCESS_EXIT " + (Get-Date -Format o))
exit 3
