param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$sourceDir = Join-Path $PSScriptRoot "../src"
$testDir = Join-Path ([System.IO.Path]::GetTempPath()) ("hechuan-tests-" + [guid]::NewGuid())
New-Item -ItemType Directory -Path $testDir | Out-Null
& node (Join-Path $PSScriptRoot "../../../host_app/scripts/profile-fixtures.mjs") $testDir
if ($LASTEXITCODE -ne 0) { throw "Profile fixture generation failed" }
$suites = @(
    @{ Name = "config"; Files = @("vcfg_protocol.c", "vcu_selection_store.c"); Test = "test_vcu_config.c" },
    @{ Name = "motion"; Files = @("hechuan_motion.c", "vcu_machine_profile.c"); Test = "test_hechuan_motion.c" },
    @{ Name = "profile"; Files = @("fgmp_profile.c", "vcfg_protocol.c"); Test = "test_fgmp_profile.c" },
    @{ Name = "bridge"; Files = @("vcu_bridge.c", "vcu_telemetry.c", "vcfg_protocol.c", "hechuan_motion.c"); Test = "test_vcu_bridge.c" }
)
foreach ($suite in $suites) {
    $binary = Join-Path $testDir ($suite.Name + ".exe")
    $sources = @($suite.Files | ForEach-Object { Join-Path $sourceDir $_ })
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic "-I$sourceDir" @sources (Join-Path $PSScriptRoot $suite.Test) -o $binary -lm
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($suite.Name)" }
    if ($suite.Name -eq "profile") { & $binary $testDir } else { & $binary }
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($suite.Name)" }
}
Write-Output "Test binaries: $testDir"
