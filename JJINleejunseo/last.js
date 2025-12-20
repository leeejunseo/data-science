import React, { useState, useEffect, useRef } from 'react';
import { Play, RotateCcw, Settings, Map, AlertCircle, Maximize, TrendingUp, Zap } from 'lucide-react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

const KoreaMissileUI = () => {
  const [launchAngle, setLaunchAngle] = useState(45);
  const [azimuth, setAzimuth] = useState(90);
  const [missileType, setMissileType] = useState('SCUD-B');
  const [isSimulating, setIsSimulating] = useState(false);
  const [simulationResults, setSimulationResults] = useState(null);
  const [currentFrame, setCurrentFrame] = useState(0);
  const canvas2dRef = useRef(null);

  const koreaMap = {
    pyongyang: { x: 0.45, y: 0.25, name: '평양', lat: 39.03, lon: 125.75 },
    wonsan: { x: 0.55, y: 0.35, name: '원산', lat: 39.15, lon: 127.44 },
    seoul: { x: 0.47, y: 0.50, name: '서울', lat: 37.56, lon: 126.97 },
    incheon: { x: 0.42, y: 0.52, name: '인천', lat: 37.45, lon: 126.69 },
    daejeon: { x: 0.48, y: 0.62, name: '대전', lat: 36.35, lon: 127.38 },
    busan: { x: 0.62, y: 0.78, name: '부산', lat: 35.17, lon: 129.07 },
  };

  const dmzLine = [
    { x: 0.35, y: 0.47 }, { x: 0.50, y: 0.45 }, { x: 0.65, y: 0.47 }
  ];

  const koreaOutline = [
    { x: 0.35, y: 0.15 }, { x: 0.40, y: 0.12 }, { x: 0.48, y: 0.10 },
    { x: 0.58, y: 0.12 }, { x: 0.65, y: 0.18 }, { x: 0.68, y: 0.25 },
    { x: 0.70, y: 0.35 }, { x: 0.68, y: 0.45 }, { x: 0.65, y: 0.55 },
    { x: 0.68, y: 0.65 }, { x: 0.65, y: 0.75 }, { x: 0.60, y: 0.82 },
    { x: 0.52, y: 0.88 }, { x: 0.45, y: 0.90 }, { x: 0.38, y: 0.88 },
    { x: 0.42, y: 0.92 }, { x: 0.38, y: 0.96 }, { x: 0.35, y: 0.90 },
    { x: 0.32, y: 0.82 }, { x: 0.30, y: 0.70 }, { x: 0.32, y: 0.60 },
    { x: 0.30, y: 0.50 }, { x: 0.32, y: 0.40 }, { x: 0.30, y: 0.30 },
    { x: 0.32, y: 0.22 }, { x: 0.35, y: 0.15 }
  ];

  const runSimulation = async () => {
    setIsSimulating(true);
    setCurrentFrame(0);

    const simTime = 600;
    const dt = 0.1;
    const elevation = launchAngle * Math.PI / 180;
    const azimuthRad = azimuth * Math.PI / 180;
    const V0 = 50;
    const g = 9.80665;

    let X = 0, Y = 0, Z = 10;
    let Vx = V0 * Math.cos(elevation) * Math.sin(azimuthRad);
    let Vy = V0 * Math.cos(elevation) * Math.cos(azimuthRad);
    let Vz = V0 * Math.sin(elevation);

    const params = {
      'SCUD-B': { mass: 3422.5, thrust: 2026728, burnTime: 65, CD: 0.25, S: 0.608 },
      'KN-23': { mass: 1750, thrust: 2176275, burnTime: 40, CD: 0.28, S: 0.709 },
      'Nodong': { mass: 8000, thrust: 5544480, burnTime: 70, CD: 0.35, S: 1.452 }
    }[missileType];

    const results = {
      time: [], x: [], y: [], h: [], velocity: [], gamma: [], psi: [], alpha: [], mach: []
    };

    let t = 0;
    let maxSteps = 6000;
    let step = 0;
    let maxAltitude = 0;
    let maxRange = 0;

    while (Z >= 0 && step < maxSteps) {
      const V = Math.sqrt(Vx**2 + Vy**2 + Vz**2);
      
      let rho;
      if (Z < 11000) {
        const T = 288.15 - 0.0065 * Z;
        const p = 101325 * Math.pow(T / 288.15, 5.2561);
        rho = p / (287.05 * T);
      } else {
        const T = 216.65;
        const p = 22632.1 * Math.exp(-0.00015768 * (Z - 11000));
        rho = p / (287.05 * T);
      }

      const qDyn = 0.5 * rho * V * V;
      const D = qDyn * params.S * params.CD;
      const dragX = -D * Vx / (V + 1e-6);
      const dragY = -D * Vy / (V + 1e-6);
      const dragZ = -D * Vz / (V + 1e-6);

      let thrustX = 0, thrustY = 0, thrustZ = 0;
      if (t <= params.burnTime) {
        const thrustMag = params.thrust;
        const theta = Math.atan2(Vz, Math.sqrt(Vx**2 + Vy**2));
        const psi = Math.atan2(Vx, Vy);
        thrustX = thrustMag * Math.cos(theta) * Math.sin(psi);
        thrustY = thrustMag * Math.cos(theta) * Math.cos(psi);
        thrustZ = thrustMag * Math.sin(theta);
      }

      const ax = (dragX + thrustX) / params.mass;
      const ay = (dragY + thrustY) / params.mass;
      const az = (dragZ + thrustZ) / params.mass - g;

      X += Vx * dt;
      Y += Vy * dt;
      Z += Vz * dt;
      Vx += ax * dt;
      Vy += ay * dt;
      Vz += az * dt;
      
      maxAltitude = Math.max(maxAltitude, Z);
      maxRange = Math.max(maxRange, Math.sqrt(X**2 + Y**2));

      if (step % 10 === 0) {
        const V_current = Math.sqrt(Vx**2 + Vy**2 + Vz**2);
        const V_horizontal = Math.sqrt(Vx**2 + Vy**2);
        const gamma = Math.atan2(Vz, V_horizontal);
        const chi = Math.atan2(Vx, Vy);
        const mach = V_current / 340;

        results.time.push(t);
        results.x.push(X);
        results.y.push(Y);
        results.h.push(Z);
        results.velocity.push(V_current);
        results.gamma.push(gamma);
        results.psi.push(chi);
        results.alpha.push(0);
        results.mach.push(mach);
      }

      t += dt;
      step++;
    }

    setSimulationResults({
      ...results,
      maxAltitude: maxAltitude,
      maxRange: maxRange,
      flightTime: t,
    });
  };

  useEffect(() => {
    if (simulationResults && isSimulating && currentFrame < simulationResults.time.length - 1) {
      const timer = setTimeout(() => {
        setCurrentFrame(prev => prev + 1);
      }, 30); // 30ms for animation speed
      return () => clearTimeout(timer);
    } else if (currentFrame >= simulationResults?.time.length - 1 && simulationResults) {
      setTimeout(() => setIsSimulating(false), 1000);
    }
  }, [simulationResults, isSimulating, currentFrame]);

  // --- 2D Visualization Effect ---
  useEffect(() => {
    const canvas = canvas2dRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    // Background and map drawing... (same as original)
    ctx.fillStyle = '#0f1e2e';
    ctx.fillRect(0, 0, width, height);
    
    ctx.fillStyle = '#2a4a3a';
    ctx.beginPath();
    koreaOutline.forEach((point, i) => {
      const x = point.x * width;
      const y = point.y * height;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = '#3a6a4a';
    ctx.lineWidth = 2;
    ctx.stroke();
    
    ctx.strokeStyle = '#ff4444';
    ctx.lineWidth = 3;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    dmzLine.forEach((point, i) => {
      const x = point.x * width;
      const y = point.y * height;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    
    Object.entries(koreaMap).forEach(([key, city]) => {
      const x = city.x * width;
      const y = city.y * height;
      ctx.fillStyle = '#ffdd44';
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(city.name, x, y - 10);
    });
    
    const launchX = koreaMap.pyongyang.x * width;
    const launchY = koreaMap.pyongyang.y * height;
    ctx.strokeStyle = '#ff0000';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(launchX, launchY, 10, 0, Math.PI * 2);
    ctx.stroke();
    
    // Missile Trajectory Drawing (same as original)
    if (simulationResults && simulationResults.x.length > 0) {
      const maxRange = Math.max(
        ...simulationResults.x.map((x, i) => Math.sqrt(x**2 + simulationResults.y[i]**2))
      );
      const scale = Math.min(width, height) * 0.35 / (maxRange + 1);
      
      ctx.strokeStyle = '#ff6600';
      ctx.lineWidth = 4;
      ctx.shadowBlur = 15;
      ctx.shadowColor = '#ff6600';
      ctx.beginPath();
      
      for (let i = 0; i <= Math.min(currentFrame, simulationResults.x.length - 1); i++) {
        const x = launchX + simulationResults.x[i] * scale;
        const y = launchY + simulationResults.y[i] * scale;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
      
      if (currentFrame < simulationResults.x.length) {
        const x = launchX + simulationResults.x[currentFrame] * scale;
        const y = launchY + simulationResults.y[currentFrame] * scale;
        
        ctx.fillStyle = '#ff0000';
        ctx.shadowBlur = 20;
        ctx.shadowColor = '#ff0000';
        ctx.beginPath();
        ctx.arc(x, y, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
        
        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
        ctx.fillRect(x + 15, y - 45, 170, 75);
        ctx.fillStyle = '#ffffff';
        ctx.font = '12px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`시간: ${simulationResults.time[currentFrame].toFixed(1)} s`, x + 20, y - 30);
        ctx.fillText(`고도: ${(simulationResults.h[currentFrame] / 1000).toFixed(1)} km`, x + 20, y - 15);
        ctx.fillText(`속도: ${simulationResults.velocity[currentFrame].toFixed(0)} m/s`, x + 20, y);
        const range = Math.sqrt(simulationResults.x[currentFrame]**2 + simulationResults.y[currentFrame]**2);
        ctx.fillText(`거리: ${(range / 1000).toFixed(1)} km`, x + 20, y + 15);
      }
    }
  }, [simulationResults, currentFrame]);

  const handleReset = () => {
    setIsSimulating(false);
    setSimulationResults(null);
    setCurrentFrame(0);
  };

  // Chart data preparation
  const chartData = simulationResults ? 
    simulationResults.time.map((t, i) => ({
      time: parseFloat(t.toFixed(1)),
      altitude: simulationResults.h[i] / 1000,
      velocity: simulationResults.velocity[i],
      mach: simulationResults.mach[i],
    })).slice(0, currentFrame + 1) : [];

  // Determine the max values for display
  const maxAltitude = simulationResults ? Math.max(...simulationResults.h) : 0;
  const maxRange = simulationResults ? Math.max(...simulationResults.x.map((x, i) => Math.sqrt(x**2 + simulationResults.y[i]**2))) : 0;
  const maxVelocity = simulationResults ? Math.max(...simulationResults.velocity) : 0;

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-blue-900 to-slate-900 p-4">
      <div className="max-w-7xl mx-auto">
        <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 mb-4 border border-slate-700">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-2xl font-bold text-white mb-1">
                🚀 한반도 6DOF 미사일 시각화
              </h1>
              <p className="text-slate-300 text-sm">
                main_visualization.py 기반 실시간 궤적 시뮬레이션
              </p>
            </div>
          </div>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-4 gap-4">
          <div className="lg:col-span-1 space-y-4">
            <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
              <div className="flex items-center gap-2 mb-3">
                <Settings className="w-5 h-5 text-blue-400" />
                <h2 className="text-lg font-semibold text-white">설정</h2>
              </div>
              
              <div className="space-y-3">
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-1">
                    미사일 종류
                  </label>
                  <select
                    value={missileType}
                    onChange={(e) => setMissileType(e.target.value)}
                    className="w-full bg-slate-700 text-white rounded px-3 py-2 text-sm border border-slate-600"
                    disabled={isSimulating}
                  >
                    <option value="SCUD-B">SCUD-B</option>
                    <option value="KN-23">KN-23</option>
                    <option value="Nodong">Nodong</option>
                  </select>
                </div>

                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-1">
                    발사각: {launchAngle}°
                  </label>
                  <input
                    type="range"
                    min="10"
                    max="85"
                    value={launchAngle}
                    onChange={(e) => setLaunchAngle(parseInt(e.target.value))}
                    className="w-full"
                    disabled={isSimulating}
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-1">
                    방위각: {azimuth}°
                  </label>
                  <input
                    type="range"
                    min="0"
                    max="360"
                    value={azimuth}
                    onChange={(e) => setAzimuth(parseInt(e.target.value))}
                    className="w-full"
                    disabled={isSimulating}
                  />
                </div>
              </div>
            </div>

            <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
              <div className="space-y-2">
                <button
                  onClick={runSimulation}
                  disabled={isSimulating}
                  className="w-full bg-gradient-to-r from-red-600 to-orange-600 hover:from-red-700 hover:to-orange-700 disabled:from-slate-600 disabled:to-slate-700 text-white font-semibold py-2 px-4 rounded transition-all flex items-center justify-center gap-2"
                >
                  <Play className="w-4 h-4" />
                  {isSimulating ? '진행 중...' : '시뮬레이션 시작'}
                </button>
                
                <button
                  onClick={handleReset}
                  className="w-full bg-slate-700 hover:bg-slate-600 text-white font-semibold py-2 px-4 rounded transition-all flex items-center justify-center gap-2"
                >
                  <RotateCcw className="w-4 h-4" />
                  초기화
                </button>
              </div>
            </div>

            {/* **완성된 부분: 시뮬레이션 결과 요약** */}
            {simulationResults && (
              <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
                <div className="flex items-center gap-2 mb-3">
                  <AlertCircle className="w-5 h-5 text-red-400" />
                  <h2 className="text-lg font-semibold text-white">결과 요약</h2>
                </div>
                <div className="text-sm space-y-2">
                  <div className='flex justify-between items-center'>
                    <span className="text-slate-400 flex items-center gap-1"><Zap className='w-4 h-4'/> 비행시간:</span>
                    <span className="text-white font-semibold">
                      {simulationResults.flightTime.toFixed(1)}초
                    </span>
                  </div>
                  <div className='flex justify-between items-center'>
                    <span className="text-slate-400 flex items-center gap-1"><Maximize className='w-4 h-4'/> 최대고도:</span>
                    <span className="text-white font-semibold">
                      {(maxAltitude / 1000).toFixed(1)} km
                    </span>
                  </div>
                  <div className='flex justify-between items-center'>
                    <span className="text-slate-400 flex items-center gap-1"><Map className='w-4 h-4'/> 최대사거리:</span>
                    <span className="text-white font-semibold">
                      {(maxRange / 1000).toFixed(1)} km
                    </span>
                  </div>
                  <div className='flex justify-between items-center'>
                    <span className="text-slate-400 flex items-center gap-1"><TrendingUp className='w-4 h-4'/> 최고속도:</span>
                    <span className="text-white font-semibold">
                      {maxVelocity.toFixed(0)} m/s
                    </span>
                  </div>
                </div>
              </div>
            )}
            {/* **완성된 부분 끝** */}

          </div>

          <div className="lg:col-span-3 space-y-4">
            <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg border border-slate-700 overflow-hidden">
              <canvas ref={canvas2dRef} width={900} height={600} className="w-full h-auto"></canvas>
            </div>

            {simulationResults && (
              <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
                <h3 className="text-lg font-semibold text-white mb-2">궤적 데이터 차트</h3>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  
                  {/* 고도/시간 차트 */}
                  <div className='bg-slate-900/50 p-3 rounded'>
                    <h4 className="text-sm font-medium text-slate-300 mb-1">고도 (km) vs 시간 (s)</h4>
                    <ResponsiveContainer width="100%" height={200}>
                      <LineChart data={chartData} margin={{ top: 5, right: 20, left: -20, bottom: 5 }}>
                        <CartesianGrid strokeDasharray="3 3" stroke="#374151" />
                        <XAxis dataKey="time" stroke="#94a3b8" tickFormatter={(t) => t.toFixed(0)} />
                        <YAxis stroke="#94a3b8" />
                        <Tooltip contentStyle={{ backgroundColor: '#1e293b', border: '1px solid #475569', color: '#ffffff' }} />
                        <Line type="monotone" dataKey="altitude" name="고도 (km)" stroke="#3b82f6" strokeWidth={2} dot={false} />
                      </LineChart>
                    </ResponsiveContainer>
                  </div>
                  
                  {/* 속도/시간 차트 */}
                  <div className='bg-slate-900/50 p-3 rounded'>
                    <h4 className="text-sm font-medium text-slate-300 mb-1">속도 (m/s) vs 시간 (s)</h4>
                    <ResponsiveContainer width="100%" height={200}>
                      <LineChart data={chartData} margin={{ top: 5, right: 20, left: -20, bottom: 5 }}>
                        <CartesianGrid strokeDasharray="3 3" stroke="#374151" />
                        <XAxis dataKey="time" stroke="#94a3b8" tickFormatter={(t) => t.toFixed(0)} />
                        <YAxis stroke="#94a3b8" />
                        <Tooltip contentStyle={{ backgroundColor: '#1e293b', border: '1px solid #475569', color: '#ffffff' }} />
                        <Line type="monotone" dataKey="velocity" name="속도 (m/s)" stroke="#ef4444" strokeWidth={2} dot={false} />
                      </LineChart>
                    </ResponsiveContainer>
                  </div>

                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default KoreaMissileUI;