@echo off
chcp 65001 > nul
cls
echo.
echo ============================================================
echo   미사일 식별 게임 - 백엔드 서버
echo ============================================================
echo.
echo 서버를 시작합니다...
echo.

cd /d "%~dp0final"

"%~dp0.venv\Scripts\python.exe" -u game_launcher.py --port 5000

echo.
echo 서버가 종료되었습니다.
pause
