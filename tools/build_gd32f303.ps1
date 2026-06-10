param(
    [string]$ParamsPath = "examples\GD32F303_FOCExplore\software\build\GD32F30X_CL\builder.params"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$paramsFullPath = Join-Path $repoRoot $ParamsPath

if (-not (Test-Path -LiteralPath $paramsFullPath)) {
    throw "builder.params not found: $paramsFullPath"
}

$extensionRoot = Join-Path $env:USERPROFILE ".vscode\extensions"
$eideExt = Get-ChildItem -LiteralPath $extensionRoot -Directory -Filter "cl.eide-*" |
    Sort-Object Name -Descending |
    Select-Object -First 1

if ($null -eq $eideExt) {
    throw "No cl.eide-* VS Code extension found under $extensionRoot"
}

$unifyBuilder = Join-Path $eideExt.FullName "res\tools\win32\unify_builder\unify_builder.exe"
$toolchainModel = Join-Path $eideExt.FullName "res\data\models\arm.v5.model.json"

if (-not (Test-Path -LiteralPath $unifyBuilder)) {
    throw "unify_builder.exe not found: $unifyBuilder"
}

if (-not (Test-Path -LiteralPath $toolchainModel)) {
    throw "ARMCC5 toolchain model not found: $toolchainModel"
}

$paramsJson = Get-Content -LiteralPath $paramsFullPath -Raw | ConvertFrom-Json
$paramsJson.toolchainCfgFile = $toolchainModel

$tempParams = Join-Path ([System.IO.Path]::GetTempPath()) ("hywfoc-builder-{0}.params" -f ([guid]::NewGuid().ToString("N")))
$paramsJson | ConvertTo-Json -Depth 16 | Set-Content -LiteralPath $tempParams -Encoding UTF8

try {
    $env:DOTNET_ROLL_FORWARD = "Major"
    & $unifyBuilder --rebuild -p $tempParams
    exit $LASTEXITCODE
}
finally {
    Remove-Item -LiteralPath $tempParams -Force -ErrorAction SilentlyContinue
}
