param(
    [ValidateRange(0.0, 100.0)]
    [double]$LossPercent,
    [switch]$Pair,
    [switch]$Headless,
    [switch]$NoRealtime
)

Set-Location $PSScriptRoot
$selectedConfig = if ($Pair) { '.\\config_opposite_pair.toml' } else { '.\\config.toml' }
$simArgs = @('.\\mode29_mujoco.py', '--config', $selectedConfig)
if ($Headless) { $simArgs += '--headless' }
if ($NoRealtime) { $simArgs += '--no-realtime' }
if ($PSBoundParameters.ContainsKey('LossPercent')) {
    $simArgs += @(
        '--loss-percent',
        $LossPercent.ToString([System.Globalization.CultureInfo]::InvariantCulture)
    )
}

python @simArgs
exit $LASTEXITCODE
