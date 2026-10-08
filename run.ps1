param(
    [ValidateRange(0.0, 100.0)]
    [double]$LossPercent
)

Set-Location $PSScriptRoot
$simArgs = @('.\mode29_mujoco.py', '--config', '.\config.toml')
if ($PSBoundParameters.ContainsKey('LossPercent')) {
    $simArgs += @(
        '--loss-percent',
        $LossPercent.ToString([System.Globalization.CultureInfo]::InvariantCulture)
    )
}

python @simArgs
exit $LASTEXITCODE
