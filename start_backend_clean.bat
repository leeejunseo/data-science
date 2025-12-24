@echo off
chcp 65001 > nul
echo ========================================
echo   백엔드 서버 시작 (ML 모델 로딩 확인)
echo ========================================
echo.

cd /d "%~dp0final"

echo ML 모델 로딩 중...
echo.

"%~dp0.venv\Scripts\python.exe" -u game_launcher.py --port 5000

pause
