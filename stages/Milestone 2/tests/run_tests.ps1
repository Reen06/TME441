param([string]$Compiler = 'c++')
$ErrorActionPreference = 'Stop'
$sourceFile = Join-Path $PSScriptRoot 'test_navigation.cpp'
$testExecutable = Join-Path ([System.IO.Path]::GetTempPath()) ('milestone2-tests-' + [guid]::NewGuid().ToString('N') + '.exe')
$compilerCommand = Get-Command $Compiler -ErrorAction Stop
$compilerPath = $compilerCommand.Source
if ([System.IO.Path]::GetFileName($compilerPath) -eq 'zig.exe') {
    $env:ZIG_LIB_DIR = Join-Path (Split-Path -Parent $compilerPath) 'lib'
    & $compilerPath c++ -std=c++11 -O2 $sourceFile -o $testExecutable
} else {
    & $compilerPath -std=c++11 -O2 $sourceFile -o $testExecutable
}
if ($LASTEXITCODE -ne 0) { throw 'Milestone 2 test compilation failed.' }
& $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Milestone 2 navigation tests failed.' }
