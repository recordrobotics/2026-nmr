param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Arguments
)

$ShScriptName = "sibling-sync.sh"
$ShScriptPath = Join-Path $PSScriptRoot $ShScriptName

if (-not (Test-Path $ShScriptPath)) {
    Write-Error "Could not find '$ShScriptName' next to this script at: $ShScriptPath"
    exit 1
}

function Find-GitBash {
    $gitCmd = Get-Command git.exe -ErrorAction SilentlyContinue
    if ($gitCmd) {
        $gitRoot = Split-Path (Split-Path $gitCmd.Source -Parent) -Parent
        $candidate = Join-Path $gitRoot "bin\bash.exe"
        if (Test-Path $candidate) { return $candidate }
    }
    return $null
}

$bashExe = Find-GitBash

if (-not $bashExe) {
    Write-Error "Could not locate Git Bash's bash.exe. Please ensure Git for Windows is installed."
    exit 1
}

Write-Verbose "Using Git Bash at: $bashExe"
Write-Verbose "Running: $ShScriptPath with arguments: $($Arguments -join ' ')"

# uncook it if windows cooked it
& $bashExe -lc "sed -i 's/\r$//' '$ShScriptPath'"

echo "bash exe: $bashExe"
echo "sh script path: $ShScriptPath"
echo "arguments: $($Arguments -join ' ')"
& $bashExe $ShScriptPath @Arguments
exit $LASTEXITCODE