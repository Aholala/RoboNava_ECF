$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$output = Join-Path $env:TEMP 'robonava_ecf_test_core.exe'
$includes = Get-ChildItem "$root\Algorithm", "$root\App", "$root\Bsp", "$root\Module" -Directory -Recurse |
    ForEach-Object { '-I' + $_.FullName }
$includes += @("-I$root\Algorithm", "-I$root\App", "-I$root\Bsp", "-I$root\Module")

& clang -std=c11 -Wall -Wextra -Werror -DBSP_LOG_DISABLED=1 @includes `
    "$PSScriptRoot\test_core.c" `
    "$root\App\app_exchange\app_exchange.c" `
    "$root\App\app_safety\app_safety.c" `
    "$root\Module\module_motor\module_motor.c" `
    "$root\Module\module_dji_motor\module_dji_motor.c" `
    "$root\Module\module_referee\module_referee_crc.c" `
    "$root\Bsp\bsp_can\bsp_can.c" `
    "$root\Algorithm\alg_crc\alg_crc.c" `
    "$root\Algorithm\alg_imu_ekf\alg_imu_ekf_core.c" `
    "$root\Algorithm\alg_imu_ekf\alg_imu_ekf_model.c" `
    "$root\Algorithm\alg_imu_ekf\alg_imu_ekf_output.c" `
    "$root\Algorithm\alg_imu_ekf\alg_imu_ekf_update.c" `
    "$root\Algorithm\alg_kalman\alg_kalman_extended.c" `
    "$root\Algorithm\alg_kalman\alg_kalman_matrix.c" `
    "$root\Algorithm\alg_filter\alg_filter_basic.c" `
    "$root\Algorithm\alg_pid\alg_pid_core.c" `
    "$root\Algorithm\alg_pid\alg_pid_angle.c" `
    "$root\Algorithm\alg_pid\alg_pid_cascade.c" `
    -o $output
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $output
exit $LASTEXITCODE
