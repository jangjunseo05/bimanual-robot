[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('status', 'start', 'stop')]
    [string]$Action,

    [Parameter(Mandatory = $true)]
    [ValidateSet('all', 'arms', 'camera', 'order')]
    [string]$Component
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

function Normalize-CommandLine {
    param([AllowNull()][string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) {
        return ''
    }
    return (($Value -replace '/', '\' -replace '\s+', ' ').Trim().ToLowerInvariant())
}

function Quote-Argument {
    param([Parameter(Mandatory = $true)][string]$Value)
    return '"' + ($Value -replace '"', '\"') + '"'
}

function Get-Processes {
    return @(Get-CimInstance -ClassName Win32_Process -Filter "Name='python.exe' OR Name='pythonw.exe' OR Name='cmd.exe'" -ErrorAction Stop)
}

function Invoke-JsonHttp {
    param(
        [Parameter(Mandatory = $true)][ValidateSet('GET', 'POST')][string]$Method,
        [Parameter(Mandatory = $true)][string]$Uri,
        [int]$TimeoutSec = 2
    )
    try {
        $response = Invoke-RestMethod -Method $Method -Uri $Uri -TimeoutSec $TimeoutSec -UseBasicParsing
        return [pscustomobject]@{ ok = $true; value = $response; error = $null }
    }
    catch {
        return [pscustomobject]@{ ok = $false; value = $null; error = $_.Exception.Message }
    }
}

function Select-Summary {
    param(
        [AllowNull()]$Value,
        [string[]]$Names
    )
    if ($null -eq $Value) {
        return $null
    }
    $summary = [ordered]@{}
    foreach ($name in $Names) {
        $property = $Value.PSObject.Properties[$name]
        if ($null -ne $property) {
            $summary[$name] = $property.Value
        }
    }
    return [pscustomobject]$summary
}

function Get-LogSummary {
    param([Parameter(Mandatory = $true)]$Role)
    $items = @()
    foreach ($entry in @(
        [pscustomobject]@{ stream = 'stdout'; path = $Role.StdoutLog },
        [pscustomobject]@{ stream = 'stderr'; path = $Role.StderrLog }
    )) {
        $exists = Test-Path -LiteralPath $entry.path -PathType Leaf
        $tail = @()
        $updated = $null
        if ($exists) {
            $file = Get-Item -LiteralPath $entry.path
            $updated = $file.LastWriteTimeUtc.ToString('o')
            $tail = @(Get-Content -LiteralPath $entry.path -Tail 12 -ErrorAction SilentlyContinue | ForEach-Object { $_.ToString() })
        }
        $items += [pscustomobject]@{
            stream = $entry.stream
            path = $entry.path
            exists = $exists
            updated_utc = $updated
            tail = $tail
        }
    }
    return $items
}

function Read-ScenarioCommand {
    param(
        [Parameter(Mandatory = $true)][string]$BatchPath,
        [Parameter(Mandatory = $true)][string]$PythonPath,
        [Parameter(Mandatory = $true)][string]$RepoPath
    )
    if (-not (Test-Path -LiteralPath $BatchPath -PathType Leaf)) {
        throw "Scenario batch file not found: $BatchPath"
    }

    $variables = @{}
    $manipLine = $null
    foreach ($line in Get-Content -LiteralPath $BatchPath) {
        if ($line -match '^\s*set\s+"?([^=\s"]+)=(.*?)(?:"\s*)?$') {
            $variables[$matches[1].ToUpperInvariant()] = $matches[2]
            continue
        }
        if ($line -match '^\s*start\s+"manip-service"\s+(.+?)\s*$') {
            $manipLine = $matches[1]
        }
    }
    if ([string]::IsNullOrWhiteSpace($manipLine)) {
        throw 'start_scenario.bat has no start "manip-service" command'
    }

    # Keep all scenario settings from the batch file, but pin Python to the
    # supported conda interpreter for the current Windows account.
    $variables['PY'] = $PythonPath
    $expanded = [regex]::Replace($manipLine, '%([^%]+)%', {
        param($match)
        $key = $match.Groups[1].Value.ToUpperInvariant()
        if (-not $variables.ContainsKey($key)) {
            throw "Undefined batch variable in manip-service command: $key"
        }
        return [string]$variables[$key]
    })

    $expanded = [regex]::Replace(
        $expanded,
        '^\s*"[^"]+"',
        (Quote-Argument -Value $PythonPath),
        [System.Text.RegularExpressions.RegexOptions]::IgnoreCase
    )
    $rolloutPath = Join-Path $RepoPath 'tools\rollout\rollout_service.py'
    $expanded = [regex]::Replace(
        $expanded,
        '(?i)(?<![\w\\])tools\\rollout\\rollout_service\.py(?![\w\\])',
        (Quote-Argument -Value $rolloutPath)
    )
    if ($expanded -notmatch '(?i)(?:^|\s)--grip\.skip_pre(?:\s|$)') {
        $expanded += ' --grip.skip_pre'
    }
    return $expanded.Trim()
}

function New-RoleDefinitions {
    param(
        [Parameter(Mandatory = $true)][string]$RepoPath,
        [Parameter(Mandatory = $true)][string]$PythonPath,
        [Parameter(Mandatory = $true)][string]$LogRoot
    )
    $batchPath = Join-Path $RepoPath 'tools\rollout\start_scenario.bat'
    $armsCommand = Read-ScenarioCommand -BatchPath $batchPath -PythonPath $PythonPath -RepoPath $RepoPath
    $cameraScript = Join-Path $RepoPath 'tools\camera\astra_rgb_virtualcam_bridge.py'
    # JDAMR_PI_ENDPOINT may override the default navigation-computer receiver.
    $piEndpoint = if ([string]::IsNullOrWhiteSpace($env:JDAMR_PI_ENDPOINT)) {
        '192.168.0.159:5600'
    }
    else {
        $env:JDAMR_PI_ENDPOINT.Trim()
    }
    if ($piEndpoint -notmatch '^[A-Za-z0-9.-]+:[0-9]{1,5}$') {
        throw "Invalid JDAMR_PI_ENDPOINT: $piEndpoint"
    }
    $cameraCommand = (Quote-Argument $PythonPath) + ' -u ' + (Quote-Argument $cameraScript) + ' --send-pi ' + $piEndpoint
    $orderCommand = (Quote-Argument $PythonPath) + ' -m apps.service_order.service_order_server --host 0.0.0.0 --port 8765 --backend rollout-service --auto-step-s 0.5'

    $definitions = [ordered]@{}
    $definitions['arms'] = [pscustomobject]@{
        Name = 'arms'
        CoreCommand = $armsCommand
        Signature = @('rollout_service.py')
        StdoutLog = Join-Path $LogRoot 'arms.stdout.log'
        StderrLog = Join-Path $LogRoot 'arms.stderr.log'
    }
    $definitions['camera'] = [pscustomobject]@{
        Name = 'camera'
        CoreCommand = $cameraCommand
        Signature = @('astra_rgb_virtualcam_bridge.py')
        StdoutLog = Join-Path $LogRoot 'camera.stdout.log'
        StderrLog = Join-Path $LogRoot 'camera.stderr.log'
    }
    $definitions['order'] = [pscustomobject]@{
        Name = 'order'
        CoreCommand = $orderCommand
        Signature = @('apps.service_order.service_order_server', '--port 8765')
        StdoutLog = Join-Path $LogRoot 'order.stdout.log'
        StderrLog = Join-Path $LogRoot 'order.stderr.log'
    }
    return $definitions
}

function Test-ContainsAll {
    param(
        [Parameter(Mandatory = $true)][string]$Haystack,
        [Parameter(Mandatory = $true)][string[]]$Needles
    )
    foreach ($needle in $Needles) {
        if (-not $Haystack.Contains((Normalize-CommandLine $needle))) {
            return $false
        }
    }
    return $true
}

function Get-RoleProcessSet {
    param(
        [Parameter(Mandatory = $true)]$Role,
        [Parameter(Mandatory = $true)][string]$RepoPath,
        [Parameter(Mandatory = $true)]$Processes
    )
    $core = Normalize-CommandLine $Role.CoreCommand
    $repo = Normalize-CommandLine $RepoPath
    $owned = @()
    $conflicts = @()
    foreach ($process in @($Processes)) {
        $line = Normalize-CommandLine $process.CommandLine
        if ([string]::IsNullOrEmpty($line)) {
            continue
        }
        $hasSignature = Test-ContainsAll -Haystack $line -Needles $Role.Signature
        if (-not $hasSignature) {
            continue
        }
        $hasCore = $line.Contains($core)
        $hasRepo = $line.Contains($repo)
        if ($hasCore -and $hasRepo) {
            $owned += $process
        }
        else {
            $conflicts += $process
        }
    }

    # A module-based Python command (the order service) does not carry its
    # working directory in CommandLine. It is owned only when its exact parent
    # is one of the repository-qualified cmd.exe wrappers above.
    $ownedWrapperIds = @($owned | Where-Object { $_.Name -ieq 'cmd.exe' } | ForEach-Object { [int]$_.ProcessId })
    if ($ownedWrapperIds.Count -gt 0) {
        $adoptedChildren = @($conflicts | Where-Object {
            $childLine = Normalize-CommandLine $_.CommandLine
            $ownedWrapperIds -contains [int]$_.ParentProcessId -and $childLine.Contains($core)
        })
        if ($adoptedChildren.Count -gt 0) {
            $owned += $adoptedChildren
            $adoptedIds = @($adoptedChildren | ForEach-Object { [int]$_.ProcessId })
            $conflicts = @($conflicts | Where-Object { $adoptedIds -notcontains [int]$_.ProcessId })
        }
    }

    $python = @($owned | Where-Object { $_.Name -ieq 'python.exe' })
    $wrappers = @($owned | Where-Object { $_.Name -ieq 'cmd.exe' })
    $primary = $null
    if ($python.Count -eq 1) {
        $primary = $python[0]
    }
    elseif ($python.Count -eq 0 -and $wrappers.Count -eq 1) {
        $primary = $wrappers[0]
    }
    $duplicate = ($python.Count -gt 1) -or ($python.Count -eq 0 -and $wrappers.Count -gt 1)
    return [pscustomobject]@{
        owned = $owned
        conflicts = $conflicts
        primary = $primary
        duplicate = $duplicate
    }
}

function Set-ExactPythonPriority {
    param([Parameter(Mandatory = $true)]$Processes)
    foreach ($process in @($Processes | Where-Object { $_.Name -ieq 'python.exe' })) {
        try {
            $null = Invoke-CimMethod -InputObject $process -MethodName SetPriority -Arguments @{ Priority = [uint32]32768 }
        }
        catch {
            throw "Could not set AboveNormal priority for PID $($process.ProcessId): $($_.Exception.Message)"
        }
    }
}

function Start-Role {
    param(
        [Parameter(Mandatory = $true)]$Role,
        [Parameter(Mandatory = $true)][string]$RepoPath
    )
    $before = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes (Get-Processes)
    if ($before.duplicate) {
        return [pscustomobject]@{ role = $Role.Name; state = 'duplicate'; started = $false; pid = $null; reason = 'multiple exact owned processes already exist' }
    }
    if (@($before.conflicts).Count -gt 0) {
        return [pscustomobject]@{
            role = $Role.Name
            state = 'conflict'
            started = $false
            pid = $(if ($null -ne $before.primary) { [int]$before.primary.ProcessId } else { $null })
            reason = 'role-like process exists without the exact repository and command identity; refusing start or adoption'
            conflicting_pids = @($before.conflicts | ForEach-Object { [int]$_.ProcessId })
        }
    }
    if ($null -ne $before.primary) {
        Set-ExactPythonPriority -Processes $before.owned
        return [pscustomobject]@{ role = $Role.Name; state = 'running'; started = $false; pid = [int]$before.primary.ProcessId; reason = 'adopted exact existing process' }
    }

    $inner = 'cd /d "{0}" && {1} 1>>"{2}" 2>>"{3}"' -f $RepoPath, $Role.CoreCommand, $Role.StdoutLog, $Role.StderrLog
    $commandLine = 'cmd.exe /d /s /c "{0}"' -f $inner
    $startup = ([WmiClass]'Win32_ProcessStartup').CreateInstance()
    $startup.ShowWindow = 0
    $startup.PriorityClass = 32768
    $processClass = [WmiClass]'Win32_Process'
    $created = $processClass.Create($commandLine, $RepoPath, $startup)
    if ([int]$created.ReturnValue -ne 0) {
        throw "Win32_Process.Create failed for $($Role.Name): return value $($created.ReturnValue)"
    }

    $deadline = [DateTime]::UtcNow.AddSeconds(5)
    $current = $null
    do {
        Start-Sleep -Milliseconds 100
        $current = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes (Get-Processes)
        if ($current.duplicate -or @($current.conflicts).Count -gt 0 -or $null -ne $current.primary) {
            break
        }
    } while ([DateTime]::UtcNow -lt $deadline)

    if ($current.duplicate) {
        return [pscustomobject]@{ role = $Role.Name; state = 'duplicate'; started = $true; pid = $null; reason = 'more than one exact process appeared after start' }
    }
    if (@($current.conflicts).Count -gt 0) {
        return [pscustomobject]@{
            role = $Role.Name
            state = 'conflict'
            started = $true
            pid = $(if ($null -ne $current.primary) { [int]$current.primary.ProcessId } else { [int]$created.ProcessId })
            reason = 'a non-exact role process appeared during start; refusing adoption'
            conflicting_pids = @($current.conflicts | ForEach-Object { [int]$_.ProcessId })
        }
    }
    if ($null -eq $current.primary) {
        return [pscustomobject]@{ role = $Role.Name; state = 'failed'; started = $true; pid = [int]$created.ProcessId; reason = 'created process did not retain the exact expected identity; inspect logs' }
    }
    Set-ExactPythonPriority -Processes $current.owned
    return [pscustomobject]@{ role = $Role.Name; state = 'running'; started = $true; pid = [int]$current.primary.ProcessId; reason = $null }
}

function Stop-ExactProcesses {
    param([Parameter(Mandatory = $true)]$ProcessSet)
    $stopped = @()
    # Stop Python children before their exact cmd.exe wrappers. No name or
    # wildcard kill is used; every PID was checked against the full role command.
    $ordered = @($ProcessSet.owned | Sort-Object @{ Expression = { if ($_.Name -ieq 'python.exe') { 0 } else { 1 } } })
    foreach ($process in $ordered) {
        $fresh = Get-CimInstance -ClassName Win32_Process -Filter ("ProcessId={0}" -f [int]$process.ProcessId) -ErrorAction SilentlyContinue
        if ($null -eq $fresh) {
            continue
        }
        $sameIdentity = (
            $fresh.Name -ieq $process.Name -and
            [int]$fresh.ParentProcessId -eq [int]$process.ParentProcessId -and
            [string]$fresh.CreationDate -eq [string]$process.CreationDate -and
            (Normalize-CommandLine $fresh.CommandLine) -ceq (Normalize-CommandLine $process.CommandLine)
        )
        if (-not $sameIdentity) {
            throw "PID $($process.ProcessId) identity changed before termination; refusing to stop it"
        }
        $result = Invoke-CimMethod -InputObject $fresh -MethodName Terminate
        if ([int]$result.ReturnValue -ne 0) {
            throw "Terminate failed for exact PID $($process.ProcessId): return value $($result.ReturnValue)"
        }
        $stopped += [int]$process.ProcessId
    }
    return $stopped
}

function Stop-ArmsRole {
    param(
        [Parameter(Mandatory = $true)]$Role,
        [Parameter(Mandatory = $true)][string]$RepoPath
    )
    $set = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes (Get-Processes)
    if ($set.duplicate) {
        return [pscustomobject]@{ role = 'arms'; state = 'manual_required'; stopped = $false; pid = $null; reason = 'duplicate exact arm services; no process was stopped' }
    }
    if (@($set.conflicts).Count -gt 0) {
        return [pscustomobject]@{
            role = 'arms'; state = 'conflict'; stopped = $false
            pid = $(if ($null -ne $set.primary) { [int]$set.primary.ProcessId } else { $null })
            reason = 'arm-like process lacks exact repository and command identity; no arm process was stopped'
            conflicting_pids = @($set.conflicts | ForEach-Object { [int]$_.ProcessId })
        }
    }
    if ($null -eq $set.primary) {
        return [pscustomobject]@{ role = 'arms'; state = 'stopped'; stopped = $false; pid = $null; reason = 'no exact owned arm service is running' }
    }

    $status = Invoke-JsonHttp -Method GET -Uri 'http://127.0.0.1:8770/status'
    if (-not $status.ok) {
        return [pscustomobject]@{
            role = 'arms'; state = 'manual_required'; stopped = $false; pid = [int]$set.primary.ProcessId
            reason = 'arm state is unavailable; refusing forced termination'; detail = $status.error
        }
    }
    $armsState = [string]$status.value.arms
    $serviceState = [string]$status.value.state
    $currentTask = $status.value.current
    if ($armsState -in @('hold', 'carrying_cup')) {
        return [pscustomobject]@{
            role = 'arms'; state = 'manual_required'; stopped = $false; pid = [int]$set.primary.ProcessId
            reason = "arms=$armsState requires an operator decision before shutdown"
        }
    }
    if ($armsState -eq 'busy' -or $serviceState -ne 'ready' -or $null -ne $currentTask) {
        $stopRequest = Invoke-JsonHttp -Method POST -Uri 'http://127.0.0.1:8770/stop' -TimeoutSec 3
        return [pscustomobject]@{
            role = 'arms'; state = 'manual_required'; stopped = $false; pid = [int]$set.primary.ProcessId
            reason = "active or queued arm work received /stop (service=$serviceState, arms=$armsState, current=$currentTask); shutdown is withheld because the resulting physical pose must be reviewed"
            stop_request = $stopRequest
        }
    }
    if ($armsState -ne 'stowed') {
        return [pscustomobject]@{
            role = 'arms'; state = 'manual_required'; stopped = $false; pid = [int]$set.primary.ProcessId
            reason = "unrecognized or unsafe arms state '$armsState'; refusing shutdown"
        }
    }

    $stopRequest = Invoke-JsonHttp -Method POST -Uri 'http://127.0.0.1:8770/stop' -TimeoutSec 3
    if (-not $stopRequest.ok) {
        return [pscustomobject]@{ role = 'arms'; state = 'failed'; stopped = $false; pid = [int]$set.primary.ProcessId; reason = '/stop request failed'; detail = $stopRequest.error }
    }
    $shutdownRequest = Invoke-JsonHttp -Method POST -Uri 'http://127.0.0.1:8770/shutdown' -TimeoutSec 3
    if (-not $shutdownRequest.ok) {
        return [pscustomobject]@{ role = 'arms'; state = 'failed'; stopped = $false; pid = [int]$set.primary.ProcessId; reason = '/shutdown request failed; process was not force-terminated'; detail = $shutdownRequest.error }
    }

    $deadline = [DateTime]::UtcNow.AddSeconds(12)
    do {
        Start-Sleep -Milliseconds 200
        $remaining = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes (Get-Processes)
        if ($null -eq $remaining.primary) {
            return [pscustomobject]@{ role = 'arms'; state = 'stopped'; stopped = $true; pid = [int]$set.primary.ProcessId; reason = 'stopped through /stop and /shutdown' }
        }
    } while ([DateTime]::UtcNow -lt $deadline)

    # A stowed service that accepted both contracts may be cleaned up, but only
    # through the PIDs whose full repository and role identity still matches.
    $remaining = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes (Get-Processes)
    $pids = Stop-ExactProcesses -ProcessSet $remaining
    return [pscustomobject]@{ role = 'arms'; state = 'stopped'; stopped = $true; pid = [int]$set.primary.ProcessId; reason = 'grace period expired after /shutdown; exact owned process was terminated'; terminated_pids = $pids }
}

function Stop-Role {
    param(
        [Parameter(Mandatory = $true)]$Role,
        [Parameter(Mandatory = $true)][string]$RepoPath
    )
    if ($Role.Name -eq 'arms') {
        return Stop-ArmsRole -Role $Role -RepoPath $RepoPath
    }
    $set = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes (Get-Processes)
    if ($set.duplicate) {
        return [pscustomobject]@{ role = $Role.Name; state = 'duplicate'; stopped = $false; pid = $null; reason = 'multiple exact owned processes; refusing ambiguous stop' }
    }
    if (@($set.conflicts).Count -gt 0) {
        return [pscustomobject]@{
            role = $Role.Name; state = 'conflict'; stopped = $false
            pid = $(if ($null -ne $set.primary) { [int]$set.primary.ProcessId } else { $null })
            reason = 'role-like process lacks exact repository and command identity; no process was stopped'
            conflicting_pids = @($set.conflicts | ForEach-Object { [int]$_.ProcessId })
        }
    }
    if ($null -eq $set.primary) {
        return [pscustomobject]@{ role = $Role.Name; state = 'stopped'; stopped = $false; pid = $null; reason = 'no exact owned process is running' }
    }
    $rolePid = [int]$set.primary.ProcessId
    $pids = Stop-ExactProcesses -ProcessSet $set
    return [pscustomobject]@{ role = $Role.Name; state = 'stopped'; stopped = $true; pid = $rolePid; reason = 'terminated exact owned process'; terminated_pids = $pids }
}

function Get-RoleStatus {
    param(
        [Parameter(Mandatory = $true)]$Role,
        [Parameter(Mandatory = $true)][string]$RepoPath,
        [Parameter(Mandatory = $true)]$Processes
    )
    $set = Get-RoleProcessSet -Role $Role -RepoPath $RepoPath -Processes $Processes
    $state = 'stopped'
    $rolePid = $null
    $priority = $null
    $reason = $null
    if ($set.duplicate) {
        $state = 'duplicate'
        $reason = 'multiple exact owned processes'
    }
    elseif (@($set.conflicts).Count -gt 0) {
        $state = 'conflict'
        $reason = 'role-like process lacks exact repository and command identity'
        if ($null -ne $set.primary) {
            $rolePid = [int]$set.primary.ProcessId
            $priority = [int]$set.primary.Priority
        }
    }
    elseif ($null -ne $set.primary) {
        $state = 'running'
        $rolePid = [int]$set.primary.ProcessId
        $priority = [int]$set.primary.Priority
    }
    return [pscustomobject]@{
        role = $Role.Name
        pid = $rolePid
        state = $state
        priority = $priority
        reason = $reason
        exact_pids = @($set.owned | ForEach-Object { [int]$_.ProcessId })
        conflicting_pids = @($set.conflicts | ForEach-Object { [int]$_.ProcessId })
        logs = Get-LogSummary -Role $Role
    }
}

$exitCode = 0
$output = $null
try {
    $repoPath = if ([string]::IsNullOrWhiteSpace($env:JDAMR_WINDOWS_REPO)) {
        Join-Path $HOME 'Desktop\프로젝트\최종프로젝트\bimanual-robot-git'
    }
    else {
        $env:JDAMR_WINDOWS_REPO
    }
    $repoPath = [System.IO.Path]::GetFullPath($repoPath)
    if (-not (Test-Path -LiteralPath $repoPath -PathType Container)) {
        throw "Repository not found: $repoPath"
    }
    $pythonPath = Join-Path $HOME 'miniconda3\envs\so101_env\python.exe'
    if (-not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) {
        throw "Python interpreter not found: $pythonPath"
    }
    $logRoot = Join-Path $HOME 'jdamr_remote\logs'
    $null = New-Item -ItemType Directory -Path $logRoot -Force
    $roles = New-RoleDefinitions -RepoPath $repoPath -PythonPath $pythonPath -LogRoot $logRoot
    $selectedNames = if ($Component -eq 'all') { @('camera', 'order', 'arms') } else { @($Component) }

    $results = @()
    if ($Action -eq 'status') {
        $processes = Get-Processes
        foreach ($name in $selectedNames) {
            $results += Get-RoleStatus -Role $roles[$name] -RepoPath $repoPath -Processes $processes
        }
    }
    elseif ($Action -eq 'start') {
        foreach ($name in $selectedNames) {
            $results += Start-Role -Role $roles[$name] -RepoPath $repoPath
        }
    }
    else {
        # Stop order intake before arms when stopping everything. The arm stop
        # still enforces its own physical-state contract independently.
        $stopNames = if ($Component -eq 'all') { @('order', 'arms', 'camera') } else { $selectedNames }
        foreach ($name in $stopNames) {
            $results += Stop-Role -Role $roles[$name] -RepoPath $repoPath
        }
    }

    $rolloutStatus = Invoke-JsonHttp -Method GET -Uri 'http://127.0.0.1:8770/status'
    $orderStatus = Invoke-JsonHttp -Method GET -Uri 'http://127.0.0.1:8765/api/state'
    $output = [ordered]@{
        ok = -not (@($results | Where-Object { $_.state -in @('failed', 'conflict', 'duplicate', 'manual_required') }).Count -gt 0)
        action = $Action
        component = $Component
        repo = $repoPath
        roles = $results
        services = [ordered]@{
            rollout_8770 = [ordered]@{
                reachable = $rolloutStatus.ok
                summary = Select-Summary -Value $rolloutStatus.value -Names @('state', 'current', 'arms', 'stow_pose', 'idle_read_failures')
                error = $rolloutStatus.error
            }
            order_8765 = [ordered]@{
                reachable = $orderStatus.ok
                summary = Select-Summary -Value $orderStatus.value -Names @('state', 'phase', 'order', 'active_order', 'backend', 'error')
                error = $orderStatus.error
            }
        }
        timestamp_utc = [DateTime]::UtcNow.ToString('o')
    }
    foreach ($result in $results) {
        $output[$result.role] = $result
    }
    if (-not $output.ok) {
        $exitCode = 2
    }
}
catch {
    $exitCode = 1
    $output = [ordered]@{
        ok = $false
        action = $Action
        component = $Component
        error = $_.Exception.Message
        timestamp_utc = [DateTime]::UtcNow.ToString('o')
    }
}

[Console]::Out.WriteLine(($output | ConvertTo-Json -Depth 10 -Compress))
exit $exitCode
