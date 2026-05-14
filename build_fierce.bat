@echo off
cd /d "%~dp0"
echo 编译凶狠版AI...
g++ -O2 -std=c++17 Amazons_simple_init.cpp -o run_fierce.exe
if %errorlevel% equ 0 (
    echo 编译成功！生成 run_fierce.exe
) else (
    echo 编译失败！
    exit /b 1
)
