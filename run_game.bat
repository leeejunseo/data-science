@echo off
chcp 65001 > nul
echo ========================================
echo   미사일 식별 게임 실행기
echo   Missile Identification Game Launcher
echo ========================================
echo.

cd /d "%~dp0"

:: Check for virtual environment
if not exist ".venv\Scripts\python.exe" (
    echo [오류] 가상환경을 찾을 수 없습니다.
    echo .venv 폴더가 있는지 확인하세요.
    pause
    exit /b 1
)

:: Start Python API Server (game_launcher.py)
echo [1/2] Python 백엔드 서버 시작 중...
echo.
echo ========================================
echo   백엔드 서버 로그
echo ========================================
echo.
cd /d "%~dp0final"
%~dp0.venv\Scripts\python.exe -u game_launcher.py --port 5000

:: Server stopped, pause before closing
pause

:: Check if node_modules exists
if not exist "lastpang\node_modules" (
    echo [1.5/2] npm 패키지 설치 중... (처음 한 번만)
    cd /d "%~dp0lastpang"
    call npm install
    cd /d "%~dp0"
)

:: Start React Frontend
echo [2/2] React 프론트엔드 시작 중...
cd /d "%~dp0lastpang"
call npm start

pause
