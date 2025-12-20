import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Play, Map as MapIcon, Target, Radar, AlertTriangle, CheckCircle, XCircle, RefreshCw, Eye, EyeOff } from 'lucide-react';

/**
 * GameApp - Missile Identification Game (Event-Driven Architecture)
 * 
 * Architecture:
 * 1. UI map animation is VISUAL ONLY (same for all missiles)
 * 2. Click missile -> Load real .npz data -> Signature analysis -> Show graphs
 * 3. Physics engine and UI are INDEPENDENT
 */

const API_BASE = 'http://localhost:5000';

const GameApp = () => {
  // Animation state (visual only - NOT synced with physics)
  const [isFlying, setIsFlying] = useState(false);
  const [flightProgress, setFlightProgress] = useState(0);  // 0 ~ 1
  
  // Game state
  const [gameMode, setGameMode] = useState('waiting'); // waiting, flying, clickable, analyzing, guessing, revealed
  const [identification, setIdentification] = useState(null);
  const [actualType, setActualType] = useState(null);
  const [userGuess, setUserGuess] = useState(null);
  const [score, setScore] = useState({ correct: 0, total: 0 });
  const [showAIHint, setShowAIHint] = useState(false);
  const [analysisMessage, setAnalysisMessage] = useState('');
  
  // Canvas refs
  const canvas2dRef = useRef(null);
  const [radarAngle, setRadarAngle] = useState(0);
  const [mapImage, setMapImage] = useState(null);

  // Missile profiles (for display only)
  const MISSILE_PROFILES = {
    'SCUD-B': { 
      altitudeRange: [80, 150],
      rangeRange: [200, 400],
      description: '표준 탄도 미사일, 단거리, 중간 고도'
    },
    'KN-23': { 
      altitudeRange: [30, 70],
      rangeRange: [400, 900],
      description: '준탄도 미사일, 저고도, 종말 Pull-up 기동'
    },
    'Nodong': { 
      altitudeRange: [200, 450],
      rangeRange: [800, 1600],
      description: '고고도 포물선 궤적, 중장거리'
    }
  };

  // Load images
  useEffect(() => {
    const img = new Image();
    img.src = '/image.png';
    img.onload = () => setMapImage(img);
  }, []);

  // Radar animation
  useEffect(() => {
    const interval = setInterval(() => {
      setRadarAngle(prev => (prev + 5) % 360);
    }, 30);
    return () => clearInterval(interval);
  }, []);

  // Flight animation (visual only - same for all missiles)
  // Speed: 0.005 per 50ms = slower animation (was 0.01 per 30ms)
  useEffect(() => {
    if (isFlying && flightProgress < 1) {
      const timer = setTimeout(() => {
        setFlightProgress(prev => Math.min(prev + 0.005, 1));  // Slower: 0.005 instead of 0.01
      }, 50);  // Slower: 50ms instead of 30ms
      return () => clearTimeout(timer);
    } else if (flightProgress >= 1 && isFlying) {
      setIsFlying(false);
      setGameMode('analyzing_ready');  // Ready for analysis button
      setAnalysisMessage('비행 완료! "분석하기" 버튼을 클릭하세요.');
    }
  }, [isFlying, flightProgress]);

  // Korea map coordinates
  const koreaMap = {
    pyongyang: { x: 0.35, y: 0.36, name: '평양' },
    seoul: { x: 0.38, y: 0.46, name: '서울' },
    busan: { x: 0.50, y: 0.76, name: '부산' },
  };

  // Start game - launch visual animation and select hidden missile
  const startGame = async () => {
    setFlightProgress(0);
    setIsFlying(true);
    setGameMode('flying');
    setUserGuess(null);
    setShowAIHint(false);
    setIdentification(null);
    setActualType(null);
    setAnalysisMessage('미사일 발사 감지...');
    
    // Call backend to select random missile (hidden)
    try {
      const response = await fetch(`${API_BASE}/api/start`);
      if (response.ok) {
        console.log('[Backend] Random missile selected (hidden)');
      }
    } catch (error) {
      // Fallback: select locally
      const missiles = Object.keys(MISSILE_PROFILES);
      const selected = missiles[Math.floor(Math.random() * missiles.length)];
      setActualType(selected);
      console.log('[Local] Selected:', selected);
    }
  };

  // Handle "예측하기" button click - trigger analysis
  const handleAnalyze = async () => {
    if (gameMode !== 'flying' && gameMode !== 'analyzing_ready') return;
    
    setIsFlying(false);  // Stop animation
    setGameMode('analyzing');
    setAnalysisMessage('시그니처 분석 중... 실제 .npz 데이터 로드 중...');
    
    try {
      // 1. Call backend to analyze real .npz data
      const response = await fetch(`${API_BASE}/api/analyze`);
      if (response.ok) {
        const data = await response.json();
        setIdentification(data.identification);
        
        // 2. Trigger main_visualization.py popup
        fetch(`${API_BASE}/api/visualize`);
        
        setGameMode('guessing');
        setAnalysisMessage('분석 완료! 그래프를 확인하고 미사일 종류를 선택하세요.');
      } else {
        throw new Error('Analysis failed');
      }
    } catch (error) {
      console.log('Backend unavailable, using local analysis');
      // Fallback: local identification based on actualType
      const localIdent = generateLocalIdentification();
      setIdentification(localIdent);
      setGameMode('guessing');
      setAnalysisMessage('분석 완료! 미사일 종류를 선택하세요.');
    }
  };

  // Legacy: Handle missile click on canvas (optional)
  const handleMissileClick = () => {
    // Canvas click is now optional - main flow uses button
    if (gameMode === 'analyzing_ready') {
      handleAnalyze();
    }
  };

  // Generate local identification (fallback)
  const generateLocalIdentification = () => {
    // Generate realistic features based on hidden missile type
    const type = actualType || 'SCUD-B';
    const profile = MISSILE_PROFILES[type];
    
    const altRange = profile.altitudeRange;
    const rngRange = profile.rangeRange;
    
    const maxAlt = altRange[0] + Math.random() * (altRange[1] - altRange[0]);
    const range = rngRange[0] + Math.random() * (rngRange[1] - rngRange[0]);
    const flightTime = 200 + range / 5;
    const hasPullup = type === 'KN-23';
    
    // Identification logic
    let predictedType = 'UNKNOWN';
    let confidence = 0;
    const reasons = [];
    
    if (maxAlt < 70) {
      predictedType = 'KN-23';
      confidence = 75;
      reasons.push(`저고도 (${maxAlt.toFixed(1)}km) - 편평 탄도`);
      if (hasPullup) {
        confidence += 20;
        reasons.push('종말 Pull-up 기동 감지!');
      }
    } else if (maxAlt > 200) {
      predictedType = 'Nodong';
      confidence = 80;
      reasons.push(`고고도 포물선 (${maxAlt.toFixed(1)}km)`);
      if (range > 800) {
        confidence += 10;
        reasons.push(`장거리 (${range.toFixed(0)}km)`);
      }
    } else {
      predictedType = 'SCUD-B';
      confidence = 70;
      reasons.push(`중간 고도 (${maxAlt.toFixed(1)}km)`);
      if (range < 400) {
        confidence += 15;
        reasons.push(`단거리 (${range.toFixed(0)}km)`);
      }
    }
    
    return {
      predicted_type: predictedType,
      confidence: Math.min(confidence, 99),
      reasons,
      features: {
        max_altitude_km: maxAlt.toFixed(1),
        range_km: range.toFixed(1),
        flight_time_s: flightTime.toFixed(1),
        has_pullup: hasPullup
      }
    };
  };

  // Handle user guess
  const handleGuess = async (guess) => {
    setUserGuess(guess);
    
    // Get actual type from backend
    if (!actualType) {
      try {
        const response = await fetch(`${API_BASE}/api/reveal`);
        if (response.ok) {
          const data = await response.json();
          setActualType(data.actual_type);
          
          // Update score
          const isCorrect = guess === data.actual_type;
          setScore(prev => ({
            correct: prev.correct + (isCorrect ? 1 : 0),
            total: prev.total + 1
          }));
          setGameMode('revealed');
          return;
        }
      } catch (error) {
        // Use local actual type
      }
    }
    
    // Local fallback
    const isCorrect = guess === actualType;
    setScore(prev => ({
      correct: prev.correct + (isCorrect ? 1 : 0),
      total: prev.total + 1
    }));
    setGameMode('revealed');
  };

  // Reset game
  const resetGame = async () => {
    setFlightProgress(0);
    setIsFlying(false);
    setGameMode('waiting');
    setIdentification(null);
    setActualType(null);
    setUserGuess(null);
    setShowAIHint(false);
    setAnalysisMessage('');
    
    try {
      await fetch(`${API_BASE}/api/reset`, { method: 'POST' });
    } catch (error) {}
  };

  // Draw radar
  useEffect(() => {
    const canvas = canvas2dRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) / 2 - 20;
    
    // Background
    ctx.fillStyle = '#001a00';
    ctx.fillRect(0, 0, width, height);
    
    // Radar circle
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fillStyle = '#002200';
    ctx.fill();
    
    // Grid
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 1;
    for (let i = 1; i <= 4; i++) {
      ctx.beginPath();
      ctx.arc(cx, cy, radius * i / 4, 0, Math.PI * 2);
      ctx.globalAlpha = 0.3;
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    
    // Crosshairs
    ctx.beginPath();
    ctx.moveTo(cx - radius, cy);
    ctx.lineTo(cx + radius, cy);
    ctx.moveTo(cx, cy - radius);
    ctx.lineTo(cx, cy + radius);
    ctx.globalAlpha = 0.3;
    ctx.stroke();
    ctx.globalAlpha = 1;
    
    // Scan line
    const scanAngle = radarAngle * Math.PI / 180;
    const scanCenterY = cy + 100;
    
    ctx.beginPath();
    ctx.moveTo(cx, scanCenterY);
    ctx.arc(cx, scanCenterY, radius, scanAngle - 0.5, scanAngle, false);
    ctx.closePath();
    const scanGradient = ctx.createRadialGradient(cx, scanCenterY, 0, cx, scanCenterY, radius);
    scanGradient.addColorStop(0, 'rgba(0, 255, 0, 0.3)');
    scanGradient.addColorStop(1, 'rgba(0, 255, 0, 0.1)');
    ctx.fillStyle = scanGradient;
    ctx.fill();
    
    ctx.beginPath();
    ctx.moveTo(cx, scanCenterY);
    ctx.lineTo(cx + radius * Math.cos(scanAngle), scanCenterY + radius * Math.sin(scanAngle));
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // Map image
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 5, 0, Math.PI * 2);
    ctx.clip();
    
    if (mapImage) {
      const imgAspect = mapImage.width / mapImage.height;
      const drawWidth = radius * 1.6;
      const drawHeight = drawWidth / imgAspect;
      
      ctx.globalAlpha = 0.6;
      ctx.filter = 'sepia(100%) saturate(300%) brightness(0.5) hue-rotate(70deg) contrast(1.2)';
      ctx.drawImage(mapImage, cx - drawWidth / 2, cy - drawHeight / 2, drawWidth, drawHeight);
      ctx.filter = 'none';
      ctx.globalAlpha = 1;
    }
    
    ctx.restore();
    
    // Visual flight animation (same for all missiles - NOT synced with physics)
    if (flightProgress > 0) {
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, radius - 5, 0, Math.PI * 2);
      ctx.clip();
      
      const imgAspect = mapImage ? mapImage.width / mapImage.height : 0.7;
      const drawWidth = radius * 1.6;
      const drawHeight = drawWidth / imgAspect;
      const imgLeft = cx - drawWidth / 2;
      const imgTop = cy - drawHeight / 2;
      
      // Launch from Pyongyang, fly south (down)
      const launchX = imgLeft + koreaMap.pyongyang.x * drawWidth;
      const launchY = imgTop + koreaMap.pyongyang.y * drawHeight;
      
      // Simple trajectory: straight down with slight curve
      // HALVED: 25% of map height (was 50%)
      const flightLength = drawHeight * 0.25;
      
      // Draw trajectory trail
      ctx.beginPath();
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 4;
      ctx.shadowColor = '#ff0000';
      ctx.shadowBlur = 15;
      
      const steps = Math.floor(flightProgress * 50);
      for (let i = 0; i <= steps; i++) {
        const t = i / 50;
        const px = launchX + Math.sin(t * Math.PI) * 20;  // Slight curve
        const py = launchY + t * flightLength;  // Move down
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
      
      // Current missile position (clickable)
      const missileX = launchX + Math.sin(flightProgress * Math.PI) * 20;
      const missileY = launchY + flightProgress * flightLength;
      
      // Draw missile
      ctx.fillStyle = gameMode === 'clickable' ? '#ffff00' : '#ff0000';
      ctx.shadowBlur = gameMode === 'clickable' ? 30 : 20;
      ctx.shadowColor = gameMode === 'clickable' ? '#ffff00' : '#ff0000';
      ctx.beginPath();
      ctx.arc(missileX, missileY, gameMode === 'clickable' ? 12 : 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.shadowBlur = 0;
      
      // "Click me" indicator when clickable
      if (gameMode === 'clickable') {
        ctx.fillStyle = '#ffffff';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('클릭!', missileX, missileY - 20);
      }
      
      ctx.restore();
    }
    
    // Border
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 3;
    ctx.stroke();
    
  }, [flightProgress, gameMode, radarAngle, mapImage]);

  // Get colors
  const getMissileColor = (type) => {
    switch (type) {
      case 'SCUD-B': return 'bg-blue-600 hover:bg-blue-700';
      case 'Nodong': return 'bg-purple-600 hover:bg-purple-700';
      case 'KN-23': return 'bg-red-600 hover:bg-red-700';
      default: return 'bg-gray-600';
    }
  };

  const getConfidenceColor = (conf) => {
    if (conf >= 80) return 'text-green-400';
    if (conf >= 60) return 'text-yellow-400';
    return 'text-red-400';
  };

  return (
    <div className="min-h-screen bg-gray-900 text-white p-4">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="text-center mb-6">
          <h1 className="text-3xl font-bold text-blue-400 mb-2">
            🎯 미사일 식별 게임
          </h1>
          <p className="text-gray-400">Missile Identification Challenge</p>
          <div className="mt-2 text-sm">
            <span className="text-green-400">Score: {score.correct}/{score.total}</span>
            {score.total > 0 && (
              <span className="text-gray-500 ml-2">
                ({Math.round(score.correct / score.total * 100)}%)
              </span>
            )}
          </div>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
          {/* Radar View */}
          <div className="lg:col-span-2 bg-gray-800 rounded-lg p-4">
            <div className="flex justify-between items-center mb-4">
              <h2 className="text-xl font-semibold flex items-center gap-2">
                <Radar size={20} className="text-green-400" />
                레이더 뷰
              </h2>
              <div className="flex gap-2">
                {/* Initial state: 시뮬레이션 시작 */}
                {gameMode === 'waiting' && (
                  <button
                    onClick={startGame}
                    className="bg-red-600 hover:bg-red-700 rounded px-4 py-2 flex items-center gap-2 font-semibold"
                  >
                    <Play size={18} /> 시뮬레이션 시작
                  </button>
                )}
                {/* Flying state: 예측하기 button appears immediately */}
                {(gameMode === 'flying' || gameMode === 'analyzing_ready') && (
                  <button
                    onClick={handleAnalyze}
                    className="bg-yellow-600 hover:bg-yellow-700 rounded px-4 py-2 flex items-center gap-2 font-semibold animate-pulse"
                  >
                    <Target size={18} /> 예측하기
                  </button>
                )}
                {/* Analyzing state */}
                {gameMode === 'analyzing' && (
                  <button
                    disabled
                    className="bg-gray-600 rounded px-4 py-2 flex items-center gap-2 font-semibold cursor-not-allowed"
                  >
                    <AlertTriangle size={18} className="animate-pulse" /> 분석 중...
                  </button>
                )}
                {/* Guessing state */}
                {gameMode === 'guessing' && (
                  <span className="bg-green-800 rounded px-4 py-2 text-sm font-semibold">
                    미사일 종류를 선택하세요!
                  </span>
                )}
                {/* Revealed: 다시 하기 */}
                {gameMode === 'revealed' && (
                  <button
                    onClick={resetGame}
                    className="bg-blue-600 hover:bg-blue-700 rounded px-4 py-2 flex items-center gap-2 font-semibold"
                  >
                    <RefreshCw size={18} /> 다시 하기
                  </button>
                )}
              </div>
            </div>
            
            <div className="relative flex justify-center">
              <canvas
                ref={canvas2dRef}
                width={600}
                height={500}
                className="rounded border border-gray-700 cursor-pointer"
                onClick={handleMissileClick}
              />
              
              {/* Status message */}
              {analysisMessage && (
                <div className="absolute top-4 left-4 bg-black/70 rounded p-2 text-sm">
                  <div className={gameMode === 'clickable' ? 'text-yellow-400 font-bold animate-pulse' : 'text-green-400'}>
                    {analysisMessage}
                  </div>
                </div>
              )}
            </div>
          </div>

          {/* Control Panel */}
          <div className="space-y-4">
            {/* Game Instructions */}
            <div className="bg-gray-800 rounded-lg p-4">
              <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
                <Target size={18} className="text-yellow-400" />
                게임 방법
              </h3>
              <div className="space-y-2 text-sm text-gray-400">
                <p>1. <span className="text-red-400">시뮬레이션 시작</span> 버튼 클릭</p>
                <p>2. 비행 중 <span className="text-yellow-400">예측하기</span> 버튼 클릭</p>
                <p>3. 물리 그래프 분석 (고도, Alpha 등)</p>
                <p>4. 시그니처 기반 미사일 종류 맞추기!</p>
              </div>
              <div className="mt-3 pt-3 border-t border-gray-700 text-xs">
                <p className="text-gray-500">힌트: KN-23은 저고도(&lt;70km) + Pull-up 기동</p>
              </div>
            </div>

            {/* Features Display */}
            {identification && identification.features && (
              <div className="bg-gray-800 rounded-lg p-4">
                <h3 className="text-lg font-semibold mb-3">탐지된 특성</h3>
                <div className="space-y-2 text-sm">
                  <div className="flex justify-between">
                    <span className="text-gray-400">최대 고도:</span>
                    <span className="font-mono">{identification.features.max_altitude_km} km</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-400">사거리:</span>
                    <span className="font-mono">{identification.features.range_km} km</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-400">비행시간:</span>
                    <span className="font-mono">{identification.features.flight_time_s} s</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-400">Pull-up 기동:</span>
                    <span className={identification.features.has_pullup ? 'text-red-400 font-bold' : 'text-gray-500'}>
                      {identification.features.has_pullup ? '감지됨!' : '없음'}
                    </span>
                  </div>
                </div>
              </div>
            )}

            {/* AI Hint */}
            {gameMode === 'guessing' && identification && (
              <div className="bg-gray-800 rounded-lg p-4">
                <div className="flex justify-between items-center mb-3">
                  <h3 className="text-lg font-semibold flex items-center gap-2">
                    <AlertTriangle size={18} className="text-yellow-400" />
                    AI 분석
                  </h3>
                  <button
                    onClick={() => setShowAIHint(!showAIHint)}
                    className="text-sm text-gray-400 hover:text-white flex items-center gap-1"
                  >
                    {showAIHint ? <EyeOff size={16} /> : <Eye size={16} />}
                    {showAIHint ? '숨기기' : '힌트 보기'}
                  </button>
                </div>
                
                {showAIHint && (
                  <div className="bg-blue-900/30 rounded p-3 border border-blue-700">
                    <div className="flex items-center justify-between mb-2">
                      <span className={`text-lg font-bold px-2 py-1 rounded ${getMissileColor(identification.predicted_type)}`}>
                        {identification.predicted_type}
                      </span>
                      <span className={`font-semibold ${getConfidenceColor(identification.confidence)}`}>
                        {identification.confidence}%
                      </span>
                    </div>
                    <div className="space-y-1">
                      {identification.reasons.map((reason, idx) => (
                        <p key={idx} className="text-xs text-gray-400">• {reason}</p>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            )}

            {/* Guess Buttons */}
            {gameMode === 'guessing' && (
              <div className="bg-gray-800 rounded-lg p-4">
                <h3 className="text-lg font-semibold mb-3">어떤 미사일일까요?</h3>
                <div className="space-y-2">
                  {['SCUD-B', 'Nodong', 'KN-23'].map((type) => (
                    <button
                      key={type}
                      onClick={() => handleGuess(type)}
                      className={`w-full py-3 px-4 rounded font-semibold transition-all ${getMissileColor(type)}`}
                    >
                      {type}
                      <span className="text-xs ml-2 opacity-70">
                        {type === 'SCUD-B' && '(단거리, 표준 탄도)'}
                        {type === 'Nodong' && '(중거리, 고고도)'}
                        {type === 'KN-23' && '(저고도, Pull-up)'}
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}

            {/* Result */}
            {gameMode === 'revealed' && (
              <div className={`rounded-lg p-4 ${userGuess === actualType ? 'bg-green-900/50 border border-green-600' : 'bg-red-900/50 border border-red-600'}`}>
                <div className="flex items-center gap-2 mb-3">
                  {userGuess === actualType ? (
                    <CheckCircle className="w-6 h-6 text-green-400" />
                  ) : (
                    <XCircle className="w-6 h-6 text-red-400" />
                  )}
                  <span className="text-xl font-bold">
                    {userGuess === actualType ? '정답!' : '오답'}
                  </span>
                </div>
                <div className="space-y-2 text-sm">
                  <div className="flex justify-between">
                    <span className="text-gray-400">실제 미사일:</span>
                    <span className={`font-bold px-2 py-0.5 rounded ${getMissileColor(actualType)}`}>
                      {actualType}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-400">당신의 선택:</span>
                    <span className="font-semibold">{userGuess}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-400">AI 예측:</span>
                    <span className="font-semibold">
                      {identification?.predicted_type}
                      {identification?.predicted_type === actualType && (
                        <span className="text-green-400 ml-1">✓</span>
                      )}
                    </span>
                  </div>
                </div>
                
                {/* Missile info */}
                <div className="mt-3 pt-3 border-t border-gray-700 text-xs text-gray-400">
                  <p>{MISSILE_PROFILES[actualType]?.description}</p>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default GameApp;
