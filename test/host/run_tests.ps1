# M1 治具 —— 主机端回归测试（不依赖硬件）
#
# 用法： powershell -ExecutionPolicy Bypass -File test/host/run_tests.ps1
# 前提： 主机上有 gcc（默认 C:\Program Files\mingw64\bin\gcc.exe）
#
# 说明： 直接编译工程里真实的 protocol.c / protocol_frame.c / bsp_sensor.c，
#        用 test/host 下的桩顶替 HAL/ADC/电机/TMC 依赖。

param(
    [string]$Gcc = "C:\Program Files\mingw64\bin\gcc.exe"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Push-Location $root
try {
    if (-not (Test-Path $Gcc)) { throw "找不到 gcc: $Gcc（可用 -Gcc 指定）" }

    foreach ($board in @("U1", "U2")) {
        Write-Host "=== 协议层 $board ==="
        $out = "test/host/${board}_test.exe"
        & $Gcc -std=c99 -Wall -Wextra -O1 -include test/host/fake_main.h `
            -I "$board/Core/Inc" "-DBOARD_$board" `
            test/host/test_protocol.c test/host/stubs.c `
            "$board/Core/Src/protocol.c" "$board/Core/Src/protocol_frame.c" -o $out
        if ($LASTEXITCODE -ne 0) { throw "$board 编译失败" }
        & $out
        if ($LASTEXITCODE -ne 0) { throw "$board 协议层测试失败" }
    }

    Write-Host "=== 对射判定 (bsp_sensor.c) ==="
    & $Gcc -std=c99 -Wall -Wextra -O1 -include test/host/fake_main.h `
        -I "U1/Core/Inc" -DBOARD_U1 `
        test/host/test_sensor.c U1/Core/Src/bsp_sensor.c -o test/host/sensor_test.exe
    if ($LASTEXITCODE -ne 0) { throw "对射测试编译失败" }
    & test/host/sensor_test.exe
    if ($LASTEXITCODE -ne 0) { throw "对射测试失败" }
}
finally {
    Pop-Location
}
