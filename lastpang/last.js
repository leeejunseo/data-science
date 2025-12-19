import React, { useState, useEffect, useRef } from 'react';
import { Play, RotateCcw, Settings, Map, AlertCircle, Box, Map as MapIcon } from 'lucide-react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

const KoreaMissileUI = () => {
  const [launchAngle, setLaunchAngle] = useState(45);
  const [azimuth, setAzimuth] = useState(90);
  const [missileType, setMissileType] = useState('SCUD-B');
  const [isSimulating, setIsSimulating] = useState(false);
  const [simulationResults, setSimulationResults] = useState(null);
  const [currentFrame, setCurrentFrame] = useState(0);
  const [viewMode, setViewMode] = useState('2d');
  const canvas2dRef = useRef(null);
  const canvas3dRef = useRef(null);
  
  const [rotation, setRotation] = useState({ x: -30, y: 45 });
  const [isDragging, setIsDragging] = useState(false);
  const [lastMouse, setLastMouse] = useState({ x: 0, y: 0 });

  const koreaMap = {
    pyongyang: { x: 0.45, y: 0.25, name: '평양' },
    wonsan: { x: 0.55, y: 0.35, name: '원산' },
    seoul: { x: 0.47, y: 0.50, name: '서울' },
    incheon: { x: 0.42, y: 0.52, name: '인천' },
    daejeon: { x: 0.48, y: 0.62, name: '대전' },
    busan: { x: 0.62, y: 0.78, name: '부산' },
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

  const project3D = (x, y, z, width, height, scale, rotX, rotY) => {
    const cx = width / 2;
    const cy = height / 2;
    const angleX = rotX * Math.PI / 180;
    const angleY = rotY * Math.PI / 180;
    
    let x1 = x * Math.cos(angleY) - z * Math.sin(angleY);
    let z1 = x * Math.sin(angleY) + z * Math.cos(angleY);
    let y1 = y;
    
    let y2 = y1 * Math.cos(angleX) - z1 * Math.sin(angleX);
    let z2 = y1 * Math.sin(angleX) + z1 * Math.cos(angleX);
    let x2 = x1;
    
    const perspective = 1000;
    const scaleProj = perspective / (perspective + z2);
    
    return {
      x: cx + x2 * scale * scaleProj,
      y: cy - y2 * scale * scaleProj,
      scale: scaleProj
    };
  };

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

    setSimulationResults(results);
  };

  useEffect(() => {
    if (simulationResults && isSimulating && currentFrame < simulationResults.time.length - 1) {
      const timer = setTimeout(() => {
        setCurrentFrame(prev => prev + 1);
      }, 30);
      return () => clearTimeout(timer);
    } else if (currentFrame >= simulationResults?.time.length - 1 && simulationResults) {
      setTimeout(() => setIsSimulating(false), 1000);
    }
  }, [simulationResults, isSimulating, currentFrame]);

  useEffect(() => {
    if (viewMode !== '2d') return;
    const canvas = canvas2dRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
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
  }, [simulationResults, currentFrame, viewMode]);

  useEffect(() => {
    if (viewMode !== '3d') return;
    const canvas = canvas3dRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    ctx.fillStyle = '#0a0e1a';
    ctx.fillRect(0, 0, width, height);
    
    const scale = 0.8;
    
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let i = -5; i <= 5; i++) {
      const p1 = project3D(i * 50000, -250000, 0, width, height, scale, rotation.x, rotation.y);
      const p2 = project3D(i * 50000, 250000, 0, width, height, scale, rotation.x, rotation.y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      
      const p3 = project3D(-250000, i * 50000, 0, width, height, scale, rotation.x, rotation.y);
      const p4 = project3D(250000, i * 50000, 0, width, height, scale, rotation.x, rotation.y);
      ctx.beginPath();
      ctx.moveTo(p3.x, p3.y);
      ctx.lineTo(p4.x, p4.y);
      ctx.stroke();
    }
    
    ctx.strokeStyle = '#3a6a4a';
    ctx.lineWidth = 2;
    ctx.beginPath();
    koreaOutline.forEach((point, i) => {
      const x = (point.x - 0.5) * 500000;
      const y = (point.y - 0.5) * 500000;
      const proj = project3D(x, y, 0, width, height, scale, rotation.x, rotation.y);
      if (i === 0) ctx.moveTo(proj.x, proj.y);
      else ctx.lineTo(proj.x, proj.y);
    });
    ctx.closePath();
    ctx.stroke();
    
    Object.entries(koreaMap).forEach(([key, city]) => {
      const x = (city.x - 0.5) * 500000;
      const y = (city.y - 0.5) * 500000;
      const proj = project3D(x, y, 0, width, height, scale, rotation.x, rotation.y);
      
      ctx.fillStyle = '#ffdd44';
      ctx.beginPath();
      ctx.arc(proj.x, proj.y, 4 * proj.scale, 0, Math.PI * 2);
      ctx.fill();
      
      ctx.fillStyle = '#ffffff';
      ctx.font = `${Math.max(10, 12 * proj.scale)}px sans-serif`;
      ctx.textAlign = 'center';
      ctx.fillText(city.name, proj.x, proj.y - 10);
    });
    
    if (simulationResults && simulationResults.x.length > 0) {
      ctx.strokeStyle = '#ff6600';
      ctx.lineWidth = 3;
      ctx.shadowBlur = 10;
      ctx.shadowColor = '#ff6600';
      ctx.beginPath();
      
      for (let i = 0; i <= Math.min(currentFrame, simulationResults.x.length - 1); i++) {
        const proj = project3D(
          simulationResults.x[i],
          simulationResults.y[i],
          simulationResults.h[i],
          width, height, scale, rotation.x, rotation.y
        );
        if (i === 0) ctx.moveTo(proj.x, proj.y);
        else ctx.lineTo(proj.x, proj.y);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
      
      if (currentFrame < simulationResults.x.length) {
        const proj = project3D(
          simulationResults.x[currentFrame],
          simulationResults.y[currentFrame],
          simulationResults.h[currentFrame],
          width, height, scale, rotation.x, rotation.y
        );
        
        ctx.fillStyle = '#ff0000';
        ctx.shadowBlur = 15;
        ctx.shadowColor = '#ff0000';
        ctx.beginPath();
        ctx.arc(proj.x, proj.y, 8 * proj.scale, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
        
        const groundProj = project3D(
          simulationResults.x[currentFrame],
          simulationResults.y[currentFrame],
          0,
          width, height, scale, rotation.x, rotation.y
        );
        ctx.strokeStyle = 'rgba(255, 0, 0, 0.3)';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(proj.x, proj.y);
        ctx.lineTo(groundProj.x, groundProj.y);
        ctx.stroke();
        ctx.setLineDash([]);
        
        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
        ctx.fillRect(proj.x + 15, proj.y - 45, 170, 75);
        ctx.fillStyle = '#ffffff';
        ctx.font = '12px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`시간: ${simulationResults.time[currentFrame].toFixed(1)} s`, proj.x + 20, proj.y - 30);
        ctx.fillText(`고도: ${(simulationResults.h[currentFrame] / 1000).toFixed(1)} km`, proj.x + 20, proj.y - 15);
        ctx.fillText(`속도: ${simulationResults.velocity[currentFrame].toFixed(0)} m/s`, proj.x + 20, proj.y);
        const range = Math.sqrt(simulationResults.x[currentFrame]**2 + simulationResults.y[currentFrame]**2);
        ctx.fillText(`거리: ${(range / 1000).toFixed(1)} km`, proj.x + 20, proj.y + 15);
      }
    }
    
    const axisLength = 100000;
    const origin = project3D(0, 0, 0, width, height, scale, rotation.x, rotation.y);
    
    const xAxis = project3D(axisLength, 0, 0, width, height, scale, rotation.x, rotation.y);
    ctx.strokeStyle = '#ff0000';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(xAxis.x, xAxis.y);
    ctx.stroke();
    ctx.fillStyle = '#ff0000';
    ctx.font = '14px sans-serif';
    ctx.fillText('X(동)', xAxis.x + 10, xAxis.y);
    
    const yAxis = project3D(0, axisLength, 0, width, height, scale, rotation.x, rotation.y);
    ctx.strokeStyle = '#00ff00';
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(yAxis.x, yAxis.y);
    ctx.stroke();
    ctx.fillStyle = '#00ff00';
    ctx.fillText('Y(북)', yAxis.x + 10, yAxis.y);
    
    const zAxis = project3D(0, 0, axisLength, width, height, scale, rotation.x, rotation.y);
    ctx.strokeStyle = '#0000ff';
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(zAxis.x, zAxis.y);
    ctx.stroke();
    ctx.fillStyle = '#0000ff';
    ctx.fillText('Z(고도)', zAxis.x + 10, zAxis.y);
    
  }, [simulationResults, currentFrame, viewMode, rotation]);

  const handleMouseDown = (e) => {
    if (viewMode !== '3d') return;
    setIsDragging(true);
    setLastMouse({ x: e.clientX, y: e.clientY });
  };

  const handleMouseMove = (e) => {
    if (!isDragging || viewMode !== '3d') return;
    const dx = e.clientX - lastMouse.x;
    const dy = e.clientY - lastMouse.y;
    setRotation(prev => ({
      x: prev.x - dy * 0.5,
      y: prev.y + dx * 0.5
    }));
    setLastMouse({ x: e.clientX, y: e.clientY });
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleReset = () => {
    setIsSimulating(false);
    setSimulationResults(null);
    setCurrentFrame(0);
  };

  const chartData = simulationResults ? 
    simulationResults.time.map((t, i) => ({
      time: t,
      altitude: simulationResults.h[i] / 1000,
      velocity: simulationResults.velocity[i],
    })).slice(0, currentFrame + 1) : [];

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-blue-900 to-slate-900 p-4">
      <div className="max-w-7xl mx-auto">
        <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 mb-4 border border-slate-700">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-2xl font-bold text-white mb-1">
                🚀 한반도 6DOF 미사일 시각화 (2D/3D)
              </h1>
              <p className="text-slate-300 text-sm">
                main_visualization.py 기반 실시간 궤적 시뮬레이션
              </p>
            </div>
            <div className="flex gap-2">
              <button
                onClick={() => setViewMode('2d')}
                className={`px-4 py-2 rounded-lg flex items-center gap-2 transition-all ${
                  viewMode === '2d' 
                    ? 'bg-blue-600 text-white' 
                    : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
                }`}
              >
                <MapIcon className="w-4 h-4" />
                2D
              </button>
              <button
                onClick={() => setViewMode('3d')}
                className={`px-4 py-2 rounded-lg flex items-center gap-2 transition-all ${
                  viewMode === '3d' 
                    ? 'bg-blue-600 text-white' 
                    : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
                }`}
              >
                <Box className="w-4 h-4" />
                3D
              </button>
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

            {simulationResults && (
              <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
                <div className="text-sm space-y-2">
                  <div>
                    <span className="text-slate-400">비행시간:</span>
                    <span className="text-white ml-2 font-semibold">
                      {simulationResults.time[simulationResults.time.length - 1].toFixed(1)}초
                    </span>
                  </div>
                  <div>
                    <span className="text-slate-400">최대고도:</span>
                    <span className="text-white ml-2 font-semibold">
                      {(Math.max(...simulationResults.h) / 1000).toFixed(1)}km
                    </span>
                  </div>
                  <div>
                    <span className="text-slate-400">최대속도:</span>
                    <span className="text-white ml-2 font-semibold">
                      {Math.max(...simulationResults.velocity).toFixed(0)}m/s
                    </span>
                  </div>
                  <div>
                    <span className="text-slate-400">최종거리:</span>
                    <span className="text-white ml-2 font-semibold">
                      {(Math.sqrt(
                        simulationResults.x[simulationResults.x.length-1]**2 + 
                        simulationResults.y[simulationResults.y.length-1]**2
                      ) / 1000).toFixed(1)}km
                    </span>
                  </div>
                </div>
              </div>
            )}

            {viewMode === '3d' && (
              <div className="bg-blue-900/30 rounded-lg p-3 border border-blue-700">
                <p className="text-xs text-blue-200">
                  🖱️ 마우스 드래그로 회전 가능
                </p>
                <p className="text-xs text-blue-300 mt-1">
                  회전: X={rotation.x.toFixed(0)}° Y={rotation.y.toFixed(0)}°
                </p>
              </div>
            )}

            <div className="bg-amber-900/30 rounded-lg p-3 border border-amber-700">
              <div className="flex items-start gap-2">
                <AlertCircle className="w-4 h-4 text-amber-400 flex-shrink-0 mt-0.5" />
                <p className="text-xs text-amber-200">교육 목적 시뮬레이터</p>
              </div>
            </div>
          </div>

          <div className="lg:col-span-3 space-y-4">
            <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
              <h2 className="text-lg font-semibold text-white mb-3">
                {viewMode === '2d' ? '한반도 지도 (2D)' : '3D 궤적 시각화'}
              </h2>
              <div 
                className="bg-slate-900 rounded-lg overflow-hidden cursor-move"
                onMouseDown={handleMouseDown}
                onMouseMove={handleMouseMove}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
              >
                {viewMode === '2d' ? (
                  <canvas
                    ref={canvas2dRef}
                    width={800}
                    height={900}
                    className="w-full h-auto"
                  />
                ) : (
                  <canvas
                    ref={canvas3dRef}
                    width={800}
                    height={800}
                    className="w-full h-auto"
                  />
                )}
              </div>
            </div>

            {chartData.length > 0 && (
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
                  <h3 className="text-sm font-semibold text-white mb-2">고도 변화</h3>
                  <ResponsiveContainer width="100%" height={200}>
                    <LineChart data={chartData}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#334155" />
                      <XAxis dataKey="time" stroke="#94a3b8" tick={{fontSize: 12}} />
                      <YAxis stroke="#94a3b8" tick={{fontSize: 12}} />
                      <Tooltip contentStyle={{backgroundColor: '#1e293b', border: 'none'}} />
                      <Line type="monotone" dataKey="altitude" stroke="#22c55e" strokeWidth={2} dot={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </div>

                <div className="bg-slate-800/50 backdrop-blur-sm rounded-lg p-4 border border-slate-700">
                  <h3 className="text-sm font-semibold text-white mb-2">속도 변화</h3>
                  <ResponsiveContainer width="100%" height={200}>
                    <LineChart data={chartData}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#334155" />
                      <XAxis dataKey="time" stroke="#94a3b8" tick={{fontSize: 12}} />
                      <YAxis stroke="#94a3b8" tick={{fontSize: 12}} />
                      <Tooltip contentStyle={{backgroundColor: '#1e293b', border: 'none'}} />
                      <Line type="monotone" dataKey="velocity" stroke="#3b82f6" strokeWidth={2} dot={false} />
                    </LineChart>
                  </ResponsiveContainer>
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