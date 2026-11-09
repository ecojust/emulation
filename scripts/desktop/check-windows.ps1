param(
    [ValidateSet("x86_64-pc-windows-msvc")]
    [string]$Target = "x86_64-pc-windows-msvc"
)

$ErrorActionPreference = "Stop"
$installers = @(Get-ChildItem "target/$Target/release/bundle/nsis/*-setup.exe")
if ($installers.Count -ne 1) { throw "Expected exactly one NSIS installer." }
$installDirectory = Join-Path ([IO.Path]::GetTempPath()) ("emulation-smoke-" + [Guid]::NewGuid().ToString("N"))
try {
    # NSIS requires /D to be last; its value extends to the end of the command line.
    $installation = Start-Process -FilePath $installers[0].FullName -ArgumentList "/S /D=$installDirectory" -Wait -PassThru
    if ($installation.ExitCode -notin @(0, 3010)) { throw "Installer failed with exit code $($installation.ExitCode)." }
    $executable = Join-Path $installDirectory "emulation.exe"
    if (-not (Test-Path $executable)) { throw "The installed native executable is missing." }
    $stream = [IO.File]::OpenRead($executable)
    $reader = [IO.BinaryReader]::new($stream)
    try {
        $stream.Position = 0x3c
        $peOffset = $reader.ReadInt32()
        $stream.Position = $peOffset
        if ($reader.ReadUInt32() -ne 0x00004550) { throw "The installed executable is not a PE image." }
        if ($reader.ReadUInt16() -ne 0x8664) { throw "The installed executable is not Windows x64." }
    } finally { $reader.Dispose() }
    foreach ($mode in @("--smoke-test", "--ui-smoke-test")) {
        $smoke = Start-Process -FilePath $executable -ArgumentList $mode -PassThru -NoNewWindow
        if (-not $smoke.WaitForExit(60000)) {
            Stop-Process -Id $smoke.Id -Force
            throw "Native client $mode timed out."
        }
        $smoke.Refresh()
        if ($smoke.ExitCode -ne 0) { throw "Native client $mode failed with exit code $($smoke.ExitCode)." }
    }
    Write-Output "Verified installed Windows x64 client, native engine and real WebView IPC smoke tests."
} finally {
    $uninstaller = Join-Path $installDirectory "uninstall.exe"
    if (Test-Path $uninstaller) { Start-Process -FilePath $uninstaller -ArgumentList "/S" -Wait | Out-Null }
    if (Test-Path $installDirectory) { Remove-Item $installDirectory -Recurse -Force }
}
