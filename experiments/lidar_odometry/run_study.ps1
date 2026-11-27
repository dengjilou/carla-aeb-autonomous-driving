param([Parameter(Mandatory)][string]$Dataset,
      [Parameter(Mandatory)][string]$Output,
      [Parameter(Mandatory)][string]$Build,
      [switch]$AwaitDownload)
$ErrorActionPreference='Stop'
$env:MPLBACKEND='Agg'
$source=(Resolve-Path $PSScriptRoot).Path
$null=New-Item -ItemType Directory -Force -Path $Output
$outPath=(Resolve-Path $Output).Path
$datasetPath=(Resolve-Path $Dataset).Path
$buildPath=(Resolve-Path $Build).Path
$linuxData=(wsl -d Ubuntu-24.04 -- wslpath -a ($datasetPath.Replace('\','/'))).Trim()
$linuxOut=(wsl -d Ubuntu-24.04 -- wslpath -a ($outPath.Replace('\','/'))).Trim()
$linuxBuild=(wsl -d Ubuntu-24.04 -- wslpath -a ($buildPath.Replace('\','/'))).Trim()
function Invoke-Sequence($seq,$mode) {
    $extra=@()
    if ($AwaitDownload) {
        $count=if ($seq -eq '05') {2761} else {1101}
        $extra=@($count,'--wait-for-download')
    }
    wsl -d Ubuntu-24.04 -- "$linuxBuild/odometry" "$linuxData/sequences/$seq/velodyne" $mode "$linuxOut/${seq}_${mode}.csv" @extra
    if ($LASTEXITCODE) { throw "ICP failed for $seq/$mode" }
}
foreach ($mode in @('identity','motion')) { Invoke-Sequence '05' $mode }
py -3.12 -B "$source/evaluate.py" $datasetPath $outPath develop
if ($LASTEXITCODE) { throw 'Development evaluation failed' }
$mode=(Get-Content -Raw -LiteralPath "$outPath/development.json" | ConvertFrom-Json).selection
Invoke-Sequence '07' $mode
py -3.12 -B "$source/evaluate.py" $datasetPath $outPath holdout
if ($LASTEXITCODE) { throw 'Holdout evaluation failed' }

