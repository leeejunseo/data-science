@echo off
chcp 65001 > nul
echo ========================================
echo   미사일 시뮬레이션 실행 스크립트
echo ========================================
echo.

cd /d "%~dp0"

if not exist "node_modules" (
    echo [1/2] 패키지 설치 중... (처음 한 번만 실행됩니다)
    call npm install
    echo.
)

echo [2/2] 서버 시작 중...
echo 브라우저가 자동으로 열립니다. (http://localhost:3000)
echo 종료하려면 이 창을 닫거나 Ctrl+C를 누르세요.
echo.
call npm start
