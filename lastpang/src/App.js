import React, { useState, useEffect, useRef } from 'react';
import { Play, Map as MapIcon } from 'lucide-react';

const KoreaMissileUI = () => {
  const [launchAngle, setLaunchAngle] = useState(45);
  const [azimuth, setAzimuth] = useState(160);
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
  const [radarAngle, setRadarAngle] = useState(0);
  const [mapImage, setMapImage] = useState(null);
  const [missileImage, setMissileImage] = useState(null);

  // 한반도 지도 이미지 로드
  useEffect(() => {
    const img = new Image();
    img.src = '/image.png';
    img.onload = () => setMapImage(img);
  }, []);

  // 미사일 이미지 로드
  useEffect(() => {
    const img = new Image();
    img.src = '/missile.svg';
    img.onload = () => setMissileImage(img);
  }, []);

  // 이미지 기준 도시 좌표 (image.png 기준으로 조정)
  const koreaMap = {
    pyongyang: { x: 0.35, y: 0.36, name: '평양' },
    seoul: { x: 0.38, y: 0.46, name: '서울' },
    cheongjin: { x: 0.54, y: 0.14, name: '청진' },
    hyesan: { x: 0.46, y: 0.18, name: '혜산' },
    busan: { x: 0.50, y: 0.76, name: '부산' },
    daegu: { x: 0.48, y: 0.66, name: '대구' },
    gwangju: { x: 0.36, y: 0.70, name: '광주' },
    daejeon: { x: 0.40, y: 0.56, name: '대전' },
  };

  const dmzLine = [
    { x: 0.32, y: 0.38 }, { x: 0.40, y: 0.36 }, { x: 0.50, y: 0.37 }, { x: 0.60, y: 0.36 }, { x: 0.68, y: 0.38 }
  ];

  // 실제 한반도 외곽선 (더 정밀한 좌표)
  const koreaOutline = [
    // 북한 북부 (두만강~압록강)
    { x: 0.70, y: 0.08 }, { x: 0.68, y: 0.06 }, { x: 0.62, y: 0.05 }, { x: 0.55, y: 0.04 },
    { x: 0.48, y: 0.05 }, { x: 0.42, y: 0.06 }, { x: 0.38, y: 0.08 }, { x: 0.32, y: 0.10 },
    { x: 0.28, y: 0.14 }, { x: 0.26, y: 0.18 },
    // 서해안 (북한)
    { x: 0.28, y: 0.22 }, { x: 0.30, y: 0.26 }, { x: 0.32, y: 0.30 }, { x: 0.30, y: 0.34 },
    // 서해안 (남한)
    { x: 0.32, y: 0.40 }, { x: 0.34, y: 0.44 }, { x: 0.32, y: 0.48 }, { x: 0.30, y: 0.52 },
    { x: 0.32, y: 0.56 }, { x: 0.30, y: 0.60 }, { x: 0.32, y: 0.64 }, { x: 0.34, y: 0.68 },
    { x: 0.36, y: 0.72 }, { x: 0.38, y: 0.76 },
    // 남해안
    { x: 0.42, y: 0.78 }, { x: 0.46, y: 0.80 }, { x: 0.50, y: 0.79 }, { x: 0.54, y: 0.78 },
    { x: 0.58, y: 0.76 }, { x: 0.62, y: 0.74 },
    // 동해안 (남한)
    { x: 0.64, y: 0.70 }, { x: 0.66, y: 0.64 }, { x: 0.68, y: 0.58 }, { x: 0.70, y: 0.52 },
    { x: 0.68, y: 0.46 }, { x: 0.70, y: 0.40 },
    // 동해안 (북한)
    { x: 0.72, y: 0.34 }, { x: 0.74, y: 0.28 }, { x: 0.76, y: 0.22 }, { x: 0.74, y: 0.16 },
    { x: 0.72, y: 0.12 }, { x: 0.70, y: 0.08 }
  ];

  // 레이더 회전 애니메이션
  useEffect(() => {
    const interval = setInterval(() => {
      setRadarAngle(prev => (prev + 5) % 360);
    }, 30);
    return () => clearInterval(interval);
  }, []);

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
    const V0 = 150;
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
      const altitude = Z * 1000;
      const { rho, T, a } = getAtmosphere(altitude);
      const mach = V / a;
      
      const gamma = Math.asin(Vz / V);
      const psi = Math.atan2(Vx, Vy);
      
      const q = 0.5 * rho * V * V;
      const D = q * params.CD * params.S;
      
      let Ax, Ay, Az;
      if (t < params.burnTime) {
        const thrustX = params.thrust * Math.cos(elevation) * Math.sin(azimuthRad) / params.mass;
        const thrustY = params.thrust * Math.cos(elevation) * Math.cos(azimuthRad) / params.mass;
        const thrustZ = params.thrust * Math.sin(elevation) / params.mass;
        
        Ax = thrustX - (D * Vx / V) / params.mass;
        Ay = thrustY - (D * Vy / V) / params.mass;
        Az = thrustZ - g - (D * Vz / V) / params.mass;
      } else {
        Ax = -(D * Vx / V) / params.mass;
        Ay = -(D * Vy / V) / params.mass;
        Az = -g - (D * Vz / V) / params.mass;
      }
      
      Vx += Ax * dt;
      Vy += Ay * dt;
      Vz += Az * dt;
      X += Vx * dt / 1000;
      Y += Vy * dt / 1000;
      Z += Vz * dt / 1000;
      
      if (step % 10 === 0) {
        results.time.push(t);
        results.x.push(X);
        results.y.push(Y);
        results.h.push(Z);
        results.velocity.push(V);
        results.gamma.push(gamma * 180 / Math.PI);
        results.psi.push(psi * 180 / Math.PI);
        results.mach.push(mach);
      }
      
      t += dt;
      step++;
    }

    setSimulationResults(results);
    setIsSimulating(false);
  };

  const getAtmosphere = (altitude) => {
    const T0 = 288.15;
    const P0 = 101325;
    const rho0 = 1.225;
    const L = 0.0065;
    const R = 287.05;
    const gamma = 1.4;
    
    let T, P, rho;
    
    if (altitude < 11000) {
      T = T0 - L * altitude;
      P = P0 * Math.pow(T / T0, 5.2561);
      rho = rho0 * Math.pow(T / T0, 4.2561);
    } else if (altitude < 25000) {
      const T11 = 216.65;
      const P11 = 22632;
      const rho11 = 0.3639;
      T = T11;
      P = P11 * Math.exp(-9.80665 * (altitude - 11000) / (R * T11));
      rho = rho11 * Math.exp(-9.80665 * (altitude - 11000) / (R * T11));
    } else {
      const T25 = 216.65 + 0.003 * (altitude - 25000);
      T = Math.max(T25, 186.87);
      rho = 0.04 * Math.exp(-altitude / 7400);
      P = rho * R * T;
    }
    
    const a = Math.sqrt(gamma * R * T);
    return { T, P, rho, a };
  };

  const draw2DMap = () => {
    const canvas = canvas2dRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) / 2 - 20;
    
    // 배경 (어두운 녹색)
    ctx.fillStyle = '#001a00';
    ctx.fillRect(0, 0, width, height);
    
    // 레이더 원형 배경
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fillStyle = '#002200';
    ctx.fill();
    
    // 레이더 그리드 원
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 1;
    for (let i = 1; i <= 4; i++) {
      ctx.beginPath();
      ctx.arc(cx, cy, radius * i / 4, 0, Math.PI * 2);
      ctx.globalAlpha = 0.3;
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    
    // 레이더 십자선
    ctx.beginPath();
    ctx.moveTo(cx - radius, cy);
    ctx.lineTo(cx + radius, cy);
    ctx.moveTo(cx, cy - radius);
    ctx.lineTo(cx, cy + radius);
    ctx.globalAlpha = 0.3;
    ctx.stroke();
    ctx.globalAlpha = 1;
    
    // 대각선
    ctx.beginPath();
    ctx.moveTo(cx - radius * 0.707, cy - radius * 0.707);
    ctx.lineTo(cx + radius * 0.707, cy + radius * 0.707);
    ctx.moveTo(cx + radius * 0.707, cy - radius * 0.707);
    ctx.lineTo(cx - radius * 0.707, cy + radius * 0.707);
    ctx.globalAlpha = 0.2;
    ctx.stroke();
    ctx.globalAlpha = 1;
    
    // 눈금 표시
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    for (let i = 0; i < 72; i++) {
      const angle = (i * 5) * Math.PI / 180;
      const innerR = i % 2 === 0 ? radius - 10 : radius - 5;
      ctx.beginPath();
      ctx.moveTo(cx + innerR * Math.cos(angle), cy + innerR * Math.sin(angle));
      ctx.lineTo(cx + radius * Math.cos(angle), cy + radius * Math.sin(angle));
      ctx.globalAlpha = 0.5;
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    
    // 회전하는 스캔 라인 (그라데이션 효과)
    const scanAngle = radarAngle * Math.PI / 180;
    const scanCenterY = cy + 100; // 스캔 중심을 아래로 이동
    const gradient = ctx.createConicalGradient ? null : ctx.createLinearGradient(cx, scanCenterY, cx + radius * Math.cos(scanAngle), scanCenterY + radius * Math.sin(scanAngle));
    
    // 스캔 영역 (부채꼴)
    ctx.beginPath();
    ctx.moveTo(cx, scanCenterY);
    ctx.arc(cx, scanCenterY, radius, scanAngle - 0.5, scanAngle, false);
    ctx.closePath();
    const scanGradient = ctx.createRadialGradient(cx, scanCenterY, 0, cx, scanCenterY, radius);
    scanGradient.addColorStop(0, 'rgba(0, 255, 0, 0.3)');
    scanGradient.addColorStop(1, 'rgba(0, 255, 0, 0.1)');
    ctx.fillStyle = scanGradient;
    ctx.fill();
    
    // 스캔 라인
    ctx.beginPath();
    ctx.moveTo(cx, scanCenterY);
    ctx.lineTo(cx + radius * Math.cos(scanAngle), scanCenterY + radius * Math.sin(scanAngle));
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // 한반도 지도 그리기 (레이더 내부에 맞춤)
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 5, 0, Math.PI * 2);
    ctx.clip();
    
    // 한반도 이미지 배경 그리기
    if (mapImage) {
      // 이미지를 레이더 원 안에 맞춤
      const imgAspect = mapImage.width / mapImage.height;
      let drawWidth, drawHeight;
      
      if (imgAspect > 1) {
        drawHeight = radius * 2;
        drawWidth = drawHeight * imgAspect;
      } else {
        drawWidth = radius * 1.6;
        drawHeight = drawWidth / imgAspect;
      }
      
      // 녹색 필터 효과를 위한 설정 (어둡게)
      ctx.globalAlpha = 0.6;
      ctx.filter = 'sepia(100%) saturate(300%) brightness(0.5) hue-rotate(70deg) contrast(1.2)';
      ctx.drawImage(
        mapImage,
        cx - drawWidth / 2,
        cy - drawHeight / 2,
        drawWidth,
        drawHeight
      );
      ctx.filter = 'none';
      ctx.globalAlpha = 1;
    } else {
      // 이미지 로드 전에는 기존 외곽선 표시
      ctx.beginPath();
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 2;
      koreaOutline.forEach((point, i) => {
        const x = cx + (point.x - 0.5) * radius * 1.8;
        const y = cy + (point.y - 0.45) * radius * 1.8;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = 'rgba(0, 100, 0, 0.3)';
      ctx.fill();
      ctx.stroke();
    }
    
    
    // 이미지 좌표계 변수 (미사일 궤적용)
    const imgAspect = mapImage ? mapImage.width / mapImage.height : 0.7;
    const drawWidth = radius * 1.6;
    const drawHeight = drawWidth / imgAspect;
    const imgLeft = cx - drawWidth / 2;
    const imgTop = cy - drawHeight / 2;
    
    ctx.restore();
    
    // 미사일 궤적
    if (simulationResults && simulationResults.x.length > 0) {
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, radius - 5, 0, Math.PI * 2);
      ctx.clip();
      
      // 이미지 좌표계에 맞춤
      const launchX = imgLeft + koreaMap.pyongyang.x * drawWidth;
      const launchY = imgTop + koreaMap.pyongyang.y * drawHeight;
      
      const maxRange = Math.max(...simulationResults.x.map((x, i) => 
        Math.sqrt(x*x + simulationResults.y[i]*simulationResults.y[i])
      ));
      const scale = radius * 0.8 / maxRange;
      
      ctx.beginPath();
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 4;
      ctx.shadowColor = '#ff0000';
      ctx.shadowBlur = 15;
      
      const frameLimit = currentFrame < simulationResults.x.length ? currentFrame : simulationResults.x.length;
      
      for (let i = 0; i < frameLimit; i++) {
        const px = launchX + simulationResults.x[i] * scale;
        const py = launchY - simulationResults.y[i] * scale;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
      
      // 미사일 현재 위치
      if (frameLimit > 0) {
        const lastIdx = frameLimit - 1;
        const missileX = launchX + simulationResults.x[lastIdx] * scale;
        const missileY = launchY - simulationResults.y[lastIdx] * scale;
        
        // 미사일 위치의 각도 계산 (스캔 중심 기준)
        const scanCenterY = cy + 100;
        const missileAngle = Math.atan2(missileY - scanCenterY, missileX - cx);
        const missileAngleDeg = ((missileAngle * 180 / Math.PI) + 360) % 360;
        const scanAngleDeg = radarAngle;
        
        // 레이더가 미사일을 지나간 후 일정 각도(150도) 동안 표시
        let angleDiff = (scanAngleDeg - missileAngleDeg + 360) % 360;
        const isVisible = angleDiff < 150;
        const fadeOpacity = isVisible ? Math.max(0, 1 - angleDiff / 150) : 0;
        
        if (isVisible && fadeOpacity > 0) {
          // 미사일 이미지 표시
          if (missileImage) {
            const missileSize = 60;
            
            // 미사일 방향 계산 (궤적 방향)
            let angle = -Math.PI / 2;
            if (lastIdx > 0) {
              const prevX = launchX + simulationResults.x[lastIdx - 1] * scale;
              const prevY = launchY - simulationResults.y[lastIdx - 1] * scale;
              angle = Math.atan2(missileY - prevY, missileX - prevX) + Math.PI / 2;
            }
            
            ctx.save();
            ctx.globalAlpha = fadeOpacity;
            ctx.translate(missileX, missileY);
            ctx.rotate(angle);
            ctx.drawImage(missileImage, -missileSize / 2, -missileSize / 2, missileSize, missileSize);
            ctx.restore();
          } else {
            // 이미지 로드 전 대체 표시
            ctx.globalAlpha = fadeOpacity;
            ctx.beginPath();
            ctx.arc(missileX, missileY, 8, 0, Math.PI * 2);
            ctx.fillStyle = '#ff0000';
            ctx.fill();
            ctx.globalAlpha = 1;
          }
          
          // 미사일 주변 링 (깜빡임 효과)
          ctx.globalAlpha = fadeOpacity * 0.6;
          ctx.beginPath();
          ctx.arc(missileX, missileY, 20 + 5 * Math.sin(Date.now() / 100), 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(255, 100, 0, 1)';
          ctx.lineWidth = 2;
          ctx.stroke();
          ctx.globalAlpha = 1;
        }
      }
      
      ctx.restore();
    }
    
    // 외곽 테두리
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 3;
    ctx.stroke();
    
    // 외곽 장식 링
    ctx.beginPath();
    ctx.arc(cx, cy, radius + 8, 0, Math.PI * 2);
    ctx.strokeStyle = '#006600';
    ctx.lineWidth = 2;
    ctx.stroke();
  };

  const draw3DView = () => {
    const canvas = canvas3dRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    ctx.fillStyle = '#0a0a1a';
    ctx.fillRect(0, 0, width, height);
    
    const gridSize = 200;
    const gridStep = 20;
    const scale = 1.5;
    
    ctx.strokeStyle = 'rgba(100, 150, 255, 0.2)';
    ctx.lineWidth = 1;
    
    for (let x = -gridSize; x <= gridSize; x += gridStep) {
      const p1 = project3D(x, 0, -gridSize, width, height, scale, rotation.x, rotation.y);
      const p2 = project3D(x, 0, gridSize, width, height, scale, rotation.x, rotation.y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    
    for (let z = -gridSize; z <= gridSize; z += gridStep) {
      const p1 = project3D(-gridSize, 0, z, width, height, scale, rotation.x, rotation.y);
      const p2 = project3D(gridSize, 0, z, width, height, scale, rotation.x, rotation.y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    
    const axisLength = 100;
    
    const origin = project3D(0, 0, 0, width, height, scale, rotation.x, rotation.y);
    const xAxis = project3D(axisLength, 0, 0, width, height, scale, rotation.x, rotation.y);
    const yAxis = project3D(0, axisLength, 0, width, height, scale, rotation.x, rotation.y);
    const zAxis = project3D(0, 0, axisLength, width, height, scale, rotation.x, rotation.y);
    
    ctx.lineWidth = 2;
    
    ctx.strokeStyle = '#ff4444';
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(xAxis.x, xAxis.y);
    ctx.stroke();
    ctx.fillStyle = '#ff4444';
    ctx.font = '14px Arial';
    ctx.fillText('X', xAxis.x + 5, xAxis.y);
    
    ctx.strokeStyle = '#44ff44';
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(yAxis.x, yAxis.y);
    ctx.stroke();
    ctx.fillStyle = '#44ff44';
    ctx.fillText('Y (고도)', yAxis.x + 5, yAxis.y);
    
    ctx.strokeStyle = '#4444ff';
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(zAxis.x, zAxis.y);
    ctx.stroke();
    ctx.fillStyle = '#4444ff';
    ctx.fillText('Z', zAxis.x + 5, zAxis.y);
    
    if (simulationResults && simulationResults.x.length > 0) {
      const maxDist = Math.max(
        Math.max(...simulationResults.x.map(Math.abs)),
        Math.max(...simulationResults.y.map(Math.abs)),
        Math.max(...simulationResults.h.map(Math.abs))
      );
      const trajScale = gridSize / maxDist * 0.8;
      
      ctx.strokeStyle = '#ffd93d';
      ctx.lineWidth = 2;
      ctx.beginPath();
      
      const frameLimit = currentFrame < simulationResults.x.length ? currentFrame : simulationResults.x.length;
      
      for (let i = 0; i < frameLimit; i++) {
        const px = simulationResults.x[i] * trajScale;
        const py = simulationResults.h[i] * trajScale;
        const pz = simulationResults.y[i] * trajScale;
        
        const projected = project3D(px, py, pz, width, height, scale, rotation.x, rotation.y);
        
        if (i === 0) ctx.moveTo(projected.x, projected.y);
        else ctx.lineTo(projected.x, projected.y);
      }
      ctx.stroke();
      
      if (frameLimit > 0) {
        const lastIdx = frameLimit - 1;
        const px = simulationResults.x[lastIdx] * trajScale;
        const py = simulationResults.h[lastIdx] * trajScale;
        const pz = simulationResults.y[lastIdx] * trajScale;
        
        const projected = project3D(px, py, pz, width, height, scale, rotation.x, rotation.y);
        
        ctx.beginPath();
        ctx.arc(projected.x, projected.y, 8, 0, Math.PI * 2);
        ctx.fillStyle = '#ff6b6b';
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }
  };

  useEffect(() => {
    if (viewMode === '2d') {
      draw2DMap();
    } else {
      draw3DView();
    }
  }, [simulationResults, currentFrame, viewMode, rotation, radarAngle, mapImage]);

  useEffect(() => {
    if (simulationResults && currentFrame < simulationResults.x.length) {
      const timer = setTimeout(() => {
        setCurrentFrame(prev => prev + 1);
      }, 20);
      return () => clearTimeout(timer);
    }
  }, [simulationResults, currentFrame]);

  const handleMouseDown = (e) => {
    if (viewMode === '3d') {
      setIsDragging(true);
      setLastMouse({ x: e.clientX, y: e.clientY });
    }
  };

  const handleMouseMove = (e) => {
    if (isDragging && viewMode === '3d') {
      const dx = e.clientX - lastMouse.x;
      const dy = e.clientY - lastMouse.y;
      setRotation(prev => ({
        x: Math.max(-90, Math.min(90, prev.x + dy * 0.5)),
        y: prev.y + dx * 0.5
      }));
      setLastMouse({ x: e.clientX, y: e.clientY });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const resetSimulation = () => {
    setSimulationResults(null);
    setCurrentFrame(0);
    setIsSimulating(false);
  };


  return (
    <div className="min-h-screen bg-gray-900 text-white p-4">
      <div className="max-w-7xl mx-auto">
        <div className="text-center mb-6">
          <h1 className="text-3xl font-bold text-blue-400 mb-2">
            🚀 한반도 미사일 시뮬레이션
          </h1>
          <p className="text-gray-400">탄도 미사일 궤적 분석 시스템</p>
        </div>

        <div className="bg-gray-800 rounded-lg p-4">
          <div className="flex justify-between items-center mb-4">
            <h2 className="text-xl font-semibold flex items-center gap-2">
              <MapIcon size={20} />
              레이더 뷰
            </h2>
            <button
              onClick={runSimulation}
              disabled={isSimulating}
              className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 rounded px-4 py-2 flex items-center gap-2 font-semibold"
            >
              <Play size={18} /> {isSimulating ? '진행 중...' : '시뮬레이션 시작'}
            </button>
          </div>
          
          <div className="relative flex justify-center">
            <canvas
              ref={canvas2dRef}
              width={600}
              height={500}
              className="rounded border border-gray-700"
            />
          </div>
        </div>
      </div>
    </div>
  );
};

export default KoreaMissileUI;
