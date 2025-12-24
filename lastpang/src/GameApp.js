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
  const [mlProcessSteps, setMlProcessSteps] = useState([]);
  const [graphImageUrl, setGraphImageUrl] = useState(null);
  const [mlDetails, setMlDetails] = useState(null);
  const [showMLVisualization, setShowMLVisualization] = useState(false);
  const [currentTreeIndex, setCurrentTreeIndex] = useState(0);
  const [featureAnimationIndex, setFeatureAnimationIndex] = useState(0);
  const [trajectoryParams, setTrajectoryParams] = useState(null);
  
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

  // Tree voting animation
  useEffect(() => {
    if (showMLVisualization && mlDetails && mlDetails.tree_predictions) {
      const interval = setInterval(() => {
        setCurrentTreeIndex(prev => (prev + 1) % mlDetails.tree_predictions.length);
      }, 500); // Change highlighted tree every 500ms
      return () => clearInterval(interval);
    }
  }, [showMLVisualization, mlDetails]);

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

  // Handle "예측하기" button click - trigger analysis and visualization simultaneously
  const handleAnalyze = async () => {
    if (gameMode !== 'flying' && gameMode !== 'analyzing_ready') return;
    
    setIsFlying(false);  // Stop animation
    setGameMode('analyzing');
    setMlProcessSteps([]);
    setGraphImageUrl(null);
    
    // Show ML processing steps
    const addStep = (step) => {
      setMlProcessSteps(prev => [...prev, { text: step, timestamp: Date.now() }]);
    };
    
    addStep('📡 NPZ 데이터 로드 중...');
    setAnalysisMessage('시그니처 분석 중... ML 모델 예측 실행 중...');
    
    await new Promise(resolve => setTimeout(resolve, 500));
    addStep('🔍 15개 특징 추출 중 (레이더 12개 + 6DOF 3개)...');
    
    try {
      await new Promise(resolve => setTimeout(resolve, 500));
      addStep('⚙️ 특징 정규화 (StandardScaler)...');
      
      // Call both APIs simultaneously (parallel execution)
      const [analyzeResponse, visualizeResponse] = await Promise.all([
        fetch(`${API_BASE}/api/analyze`),
        fetch(`${API_BASE}/api/visualize`)
      ]);
      
      await new Promise(resolve => setTimeout(resolve, 500));
      addStep('🤖 RandomForest 모델 추론 중...');
      
      if (analyzeResponse.ok) {
        const data = await analyzeResponse.json();
        
        // Debug: Log the full response
        console.log('=== API Response ===', data);
        console.log('identification:', data.identification);
        console.log('ml_details:', data.identification?.ml_details);
        
        // Store ML details for visualization
        if (data.identification && data.identification.ml_details) {
          console.log('✅ ML details found, setting state');
          setMlDetails(data.identification.ml_details);
          setShowMLVisualization(true);
        } else {
          console.warn('⚠️ No ml_details in response');
        }
        
        await new Promise(resolve => setTimeout(resolve, 500));
        addStep('📊 확률 분포 계산 완료');
        addStep(`✅ 예측 완료: ${data.identification.predicted_type} (신뢰도: ${data.identification.confidence}%)`);
        
        setIdentification(data.identification);
        
        // Set graph image if available
        if (data.graph_image) {
          setGraphImageUrl(`data:image/png;base64,${data.graph_image}`);
          addStep('📈 그래프 생성 완료');
        }
        
        // Set actual missile type (but don't show yet)
        if (data.actual_missile) {
          setActualType(data.actual_missile);
        }
        
        // Store trajectory parameters
        if (data.trajectory_params) {
          setTrajectoryParams(data.trajectory_params);
        }
        
        // ML이 자동으로 예측 완료 - 바로 정답 확인
        setGameMode('ml_predicting');
        setAnalysisMessage('ML 모델 분석 완료! 예측 결과를 확인하세요.');
        
        // 자동으로 ML 예측을 정답으로 제출
        setTimeout(() => {
          handleMLPrediction(data.identification.predicted_type);
        }, 8000); // 8초 후 자동 제출 (ML 시각화 시간 확보)
        
        if (visualizeResponse.ok) {
          console.log('✓ Visualization launched successfully');
        }
      } else {
        throw new Error('Analysis failed');
      }
    } catch (error) {
      console.log('Backend unavailable, using local analysis');
      addStep('⚠️ 백엔드 연결 실패, 로컬 분석 사용');
      const localIdent = generateLocalIdentification();
      setIdentification(localIdent);
      setGameMode('ml_predicting');
      setAnalysisMessage('ML 모델 분석 완료! 예측 결과를 확인하세요.');
      setTimeout(() => {
        handleMLPrediction(localIdent.predicted_type);
      }, 3000);
    }
  };

  // Handle ML prediction (automatic)
  const handleMLPrediction = async (mlPrediction) => {
    setUserGuess(mlPrediction);
    
    // Get actual type from backend
    try {
      const response = await fetch(`${API_BASE}/api/reveal`);
      if (response.ok) {
        const data = await response.json();
        setActualType(data.actual_type);
        
        // Update score based on ML prediction
        const isCorrect = mlPrediction === data.actual_type;
        setScore(prev => ({
          correct: prev.correct + (isCorrect ? 1 : 0),
          total: prev.total + 1
        }));
        setGameMode('revealed');
        return;
      }
    } catch (error) {
      console.log('Error revealing actual type');
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
    setMlProcessSteps([]);
    setGraphImageUrl(null);
    setMlDetails(null);
    setShowMLVisualization(false);
    setCurrentTreeIndex(0);
    setFeatureAnimationIndex(0);
    setTrajectoryParams(null);
    
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
                {/* Flying state: disabled button */}
                {gameMode === 'flying' && (
                  <button
                    disabled
                    className="bg-gray-600 rounded px-4 py-2 flex items-center gap-2 font-semibold cursor-not-allowed opacity-50"
                  >
                    <Target size={18} /> 비행 중...
                  </button>
                )}
                {/* Ready for analysis after landing */}
                {gameMode === 'analyzing_ready' && (
                  <button
                    onClick={handleAnalyze}
                    className="bg-yellow-600 hover:bg-yellow-700 rounded px-4 py-2 flex items-center gap-2 font-semibold animate-pulse"
                  >
                    <Target size={18} /> 분석하기
                  </button>
                )}
                {/* Analyzing state */}
                {gameMode === 'analyzing' && (
                  <button
                    disabled
                    className="bg-gray-600 rounded px-4 py-2 flex items-center gap-2 font-semibold cursor-not-allowed"
                  >
                    <AlertTriangle size={18} className="animate-pulse" /> ML 분석 중...
                  </button>
                )}
                {/* ML Predicting state */}
                {gameMode === 'ml_predicting' && (
                  <button
                    disabled
                    className="bg-purple-600 rounded px-4 py-2 flex items-center gap-2 font-semibold cursor-not-allowed"
                  >
                    <AlertTriangle size={18} className="animate-pulse" /> ML 예측 완료
                  </button>
                )}
                {/* Guessing state (legacy - not used in ML mode) */}
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
              {/* Show graph if available, otherwise show radar */}
              {graphImageUrl ? (
                <div className="w-full">
                  <img 
                    src={graphImageUrl} 
                    alt="Missile Trajectory Visualization" 
                    className="w-full rounded border border-gray-700"
                  />
                </div>
              ) : (
                <canvas
                  ref={canvas2dRef}
                  width={600}
                  height={500}
                  className="rounded border border-gray-700 cursor-pointer"
                  onClick={handleMissileClick}
                />
              )}
              
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
              <div className="space-y-1.5 text-sm text-gray-400">
                <p>1. <span className="text-red-400">시뮬레이션 시작</span> 버튼 클릭</p>
                <p>2. <span className="text-yellow-400">예측하기</span> 버튼 클릭</p>
                <p>3. <span className="text-green-400">6DOF 시뮬레이션</span> 자동 실행</p>
                <p>4. <span className="text-purple-400">ML 자동 분석</span> 및 예측</p>
                <p>5. 그래프 + 분석 결과 확인</p>
                <p>6. ML 정확도 평가!</p>
              </div>
              <div className="mt-3 pt-3 border-t border-gray-700 text-xs">
                <p className="text-gray-500">💡 실시간 생성: 고각 15~80°, 방위각 90° 고정</p>
                <p className="text-gray-500">💡 KN-23: 저고도(&lt;70km) + Pull-up 기동</p>
              </div>
            </div>

            {/* ML Processing Steps */}
            {(gameMode === 'analyzing' || gameMode === 'ml_predicting') && mlProcessSteps.length > 0 && (
              <div className="bg-gray-800 rounded-lg p-4 border border-cyan-500">
                <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
                  <AlertTriangle size={18} className="text-cyan-400 animate-pulse" />
                  ML 처리 과정
                </h3>
                <div className="space-y-2 max-h-48 overflow-y-auto">
                  {mlProcessSteps.map((step, idx) => (
                    <div 
                      key={idx} 
                      className="text-sm text-gray-300 bg-black/30 rounded px-3 py-2 animate-fadeIn"
                      style={{ animationDelay: `${idx * 0.1}s` }}
                    >
                      {step.text}
                    </div>
                  ))}
                </div>
              </div>
            )}

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

            {/* ML Analysis Results - Always visible when available */}
            {(gameMode === 'ml_predicting' || gameMode === 'revealed') && identification && (
              <div className="bg-gradient-to-br from-purple-900/50 to-blue-900/50 rounded-lg p-4 border border-purple-500">
                <div className="mb-3">
                  <h3 className="text-lg font-semibold flex items-center gap-2 mb-2">
                    <AlertTriangle size={18} className="text-purple-400" />
                    ML 모델 예측 결과
                  </h3>
                  <div className="text-xs text-gray-400">
                    {identification.method || 'RandomForest 15-feature model'}
                  </div>
                </div>
                
                <div className="bg-black/30 rounded p-3 mb-3">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm text-gray-400">ML 예측:</span>
                    <span className={`text-xl font-bold px-3 py-1 rounded ${getMissileColor(identification.predicted_type)}`}>
                      {identification.predicted_type}
                    </span>
                  </div>
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm text-gray-400">신뢰도:</span>
                    <span className={`text-lg font-semibold ${getConfidenceColor(identification.confidence)}`}>
                      {identification.confidence}%
                    </span>
                  </div>
                  {actualType && (
                    <div className="mt-3 pt-3 border-t border-purple-700/50">
                      <div className="flex items-center justify-between mb-2">
                        <span className="text-sm text-gray-400">실제 미사일:</span>
                        <span className={`text-2xl font-bold px-4 py-2 rounded ${getMissileColor(actualType)} ${gameMode === 'revealed' ? 'animate-pulse' : ''}`}>
                          {actualType}
                        </span>
                      </div>
                      {gameMode === 'revealed' && (
                        <>
                          <div className="flex items-center justify-between mt-2">
                            <span className="text-sm text-gray-400">ML 예측 결과:</span>
                            <span className={`text-xl font-bold ${identification.predicted_type === actualType ? 'text-green-400' : 'text-red-400'}`}>
                              {identification.predicted_type === actualType ? '✅ 정확!' : '❌ 오류'}
                            </span>
                          </div>
                          {trajectoryParams && (
                            <div className="mt-3 pt-3 border-t border-purple-700/50">
                              <div className="text-xs font-semibold text-cyan-300 mb-2">🚀 발사 파라미터</div>
                              <div className="grid grid-cols-2 gap-2 text-xs">
                                <div className="bg-black/30 rounded p-2">
                                  <div className="text-gray-400">발사각 (Elevation)</div>
                                  <div className="text-cyan-400 font-bold text-lg">{trajectoryParams.launch_angle}°</div>
                                </div>
                                <div className="bg-black/30 rounded p-2">
                                  <div className="text-gray-400">방위각 (Azimuth)</div>
                                  <div className="text-cyan-400 font-bold text-lg">{trajectoryParams.azimuth}°</div>
                                </div>
                                {trajectoryParams.seed !== null && (
                                  <div className="bg-black/30 rounded p-2 col-span-2">
                                    <div className="text-gray-400">시뮬레이션 Seed</div>
                                    <div className="text-cyan-400 font-mono">{trajectoryParams.seed}</div>
                                  </div>
                                )}
                              </div>
                            </div>
                          )}
                        </>
                      )}
                    </div>
                  )}
                </div>

                {/* Detailed ML Visualization */}
                {mlDetails && showMLVisualization && (
                  <div className="mt-4 space-y-3 border-t border-purple-700/50 pt-3">
                    <div className="text-sm font-semibold text-purple-300 mb-2">
                      🌲 RandomForest 분석 과정 (총 {mlDetails.n_estimators}개 트리)
                    </div>
                    
                    {/* Feature Importance Visualization - TOP 10 like eval_by_angle.py */}
                    <div className="bg-black/30 rounded p-3">
                      <div className="text-xs font-semibold text-blue-300 mb-2">
                        📊 특징 중요도 (Top 10) - eval_by_angle.py 형식
                      </div>
                      <div className="space-y-1 max-h-64 overflow-y-auto">
                        {mlDetails.detailed_features && mlDetails.detailed_features.slice(0, 10).map((feature, idx) => (
                          <div key={idx} className="text-xs">
                            <div className="flex justify-between mb-1">
                              <div className="flex items-center gap-2">
                                <span className="text-cyan-400 font-bold w-6">{idx + 1}.</span>
                                <span className="text-gray-300">{feature.name}</span>
                              </div>
                              <span className="text-yellow-400 font-mono">{feature.importance.toFixed(2)}%</span>
                            </div>
                            <div className="w-full bg-gray-700 rounded-full h-2 ml-8">
                              <div 
                                className="bg-gradient-to-r from-yellow-500 to-orange-500 h-2 rounded-full transition-all duration-1000"
                                style={{ width: `${Math.min(feature.importance, 100)}%` }}
                              />
                            </div>
                            <div className="flex justify-between mt-1 text-[10px] text-gray-500 ml-8">
                              <span>원본: {feature.raw_value}</span>
                              <span>정규화: {feature.scaled_value}</span>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>

                    {/* Tree Voting Visualization */}
                    {mlDetails.tree_predictions && mlDetails.tree_predictions.length > 0 && (
                      <div className="bg-black/30 rounded p-3">
                        <div className="text-xs font-semibold text-green-300 mb-2">
                          🗳️ 결정 트리 투표 (샘플 10개)
                        </div>
                        <div className="grid grid-cols-5 gap-1 mb-2">
                          {mlDetails.tree_predictions.map((tree, idx) => (
                            <div 
                              key={idx}
                              className={`text-center p-1 rounded text-[10px] font-bold transition-all duration-300 ${
                                getMissileColor(tree.prediction)
                              } ${idx === currentTreeIndex ? 'ring-2 ring-white scale-110' : 'opacity-60'}`}
                            >
                              T{tree.tree_id}
                            </div>
                          ))}
                        </div>
                        <div className="text-xs text-gray-400 mb-2">
                          투표 집계:
                        </div>
                        <div className="space-y-1">
                          {mlDetails.voting_summary && Object.entries(mlDetails.voting_summary).map(([missile, votes]) => (
                            <div key={missile} className="flex items-center gap-2">
                              <span className={`text-xs font-bold px-2 py-1 rounded ${getMissileColor(missile)}`}>
                                {missile}
                              </span>
                              <div className="flex-1 bg-gray-700 rounded-full h-4">
                                <div 
                                  className={`h-4 rounded-full transition-all duration-1000 flex items-center justify-end pr-1 ${
                                    missile === identification.predicted_type ? 'bg-green-500' : 'bg-gray-500'
                                  }`}
                                  style={{ width: `${(votes / 10) * 100}%` }}
                                >
                                  <span className="text-[10px] font-bold text-white">{votes}/10</span>
                                </div>
                              </div>
                            </div>
                          ))}
                        </div>
                      </div>
                    )}

                    {/* Probability Distribution */}
                    <div className="bg-black/30 rounded p-3">
                      <div className="text-xs font-semibold text-pink-300 mb-2">
                        📈 최종 확률 분포
                      </div>
                      <div className="space-y-2">
                        {identification.all_probabilities && Object.entries(identification.all_probabilities)
                          .sort(([,a], [,b]) => b - a)
                          .map(([missile, prob]) => (
                            <div key={missile}>
                              <div className="flex justify-between mb-1">
                                <span className={`text-xs font-bold ${getMissileColor(missile)}`}>
                                  {missile}
                                </span>
                                <span className="text-xs font-mono text-yellow-400">{prob}%</span>
                              </div>
                              <div className="w-full bg-gray-700 rounded-full h-3">
                                <div 
                                  className={`h-3 rounded-full transition-all duration-1000 ${
                                    missile === identification.predicted_type 
                                      ? 'bg-gradient-to-r from-green-500 to-emerald-500' 
                                      : 'bg-gradient-to-r from-gray-500 to-gray-600'
                                  }`}
                                  style={{ width: `${prob}%` }}
                                />
                              </div>
                            </div>
                          ))}
                      </div>
                    </div>
                  </div>
                )}

                <div className="space-y-2">
                  <div className="text-xs font-semibold text-purple-300 mb-1">분석 근거:</div>
                  {identification.reasons && identification.reasons.map((reason, idx) => (
                    <div key={idx} className="text-xs text-gray-300 bg-black/20 rounded px-2 py-1">
                      • {reason}
                    </div>
                  ))}
                </div>

                {identification.all_probabilities && (
                  <div className="mt-3 pt-3 border-t border-purple-700/50">
                    <div className="text-xs font-semibold text-purple-300 mb-2">전체 확률 분포:</div>
                    <div className="space-y-1">
                      {Object.entries(identification.all_probabilities).map(([type, prob]) => (
                        <div key={type} className="flex items-center gap-2">
                          <span className="text-xs text-gray-400 w-16">{type}:</span>
                          <div className="flex-1 bg-gray-700 rounded-full h-2">
                            <div 
                              className={`h-2 rounded-full ${type === identification.predicted_type ? 'bg-purple-500' : 'bg-gray-500'}`}
                              style={{ width: `${prob}%` }}
                            />
                          </div>
                          <span className="text-xs text-gray-400 w-12 text-right">{prob}%</span>
                        </div>
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
                    {userGuess === actualType ? 'ML 모델 정답!' : 'ML 모델 오답'}
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
                    <span className="text-gray-400">ML 예측:</span>
                    <span className="font-semibold">{userGuess}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-400">ML 정확도:</span>
                    <span className={`font-semibold ${userGuess === actualType ? 'text-green-400' : 'text-red-400'}`}>
                      {userGuess === actualType ? '✓ 정확' : '✗ 오류'}
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
