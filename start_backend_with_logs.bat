@echo off
chcp 65001 > nul
echo ========================================
echo   백엔드 서버 (로그 표시)
echo ========================================
echo.

cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo [오류] 가상환경을 찾을 수 없습니다.
    pause
    exit /b 1
)

echo ML 분석 로그가 이 창에 표시됩니다.
echo.
echo ========================================
echo.

cd /d "%~dp0final"
%~dp0.venv\Scripts\python.exe -u game_launcher.py --port 5000

pause
