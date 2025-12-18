import React, { useEffect, useRef } from 'react';

// Cesium은 CDN에서 로드됨 (window.Cesium)
const Cesium = window.Cesium;

// Cesium Ion 기본 토큰 (무료 계정으로 발급 가능)
if (Cesium) {
  Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlYWE1OWUxNy1mMWZiLTQzYjYtYTQ0OS1kMWFjYmFkNjc5YzciLCJpZCI6NTc1MzcsImlhdCI6MTYyMjY0NDE4Mn0.XcKpgANiY19MC4bdFUXMVEBToBmqS8kuYpUlxJHYZxk';
}

const CesiumView = ({ simulationResults, currentFrame, pyongyangCoords }) => {
  const cesiumContainerRef = useRef(null);
  const viewerRef = useRef(null);
  const trajectoryEntityRef = useRef(null);
  const missileEntityRef = useRef(null);

  // 평양 좌표 (위도, 경도)
  const PYONGYANG = {
    lat: 39.0392,
    lon: 125.7625
  };

  useEffect(() => {
    if (!cesiumContainerRef.current || !Cesium) return;

    // Cesium Viewer 생성 (Esri 위성 이미지 사용)
    const viewer = new Cesium.Viewer(cesiumContainerRef.current, {
      imageryProvider: new Cesium.ArcGisMapServerImageryProvider({
        url: 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer'
      }),
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      animation: false,
      timeline: false,
      fullscreenButton: false,
      vrButton: false,
      infoBox: false,
      selectionIndicator: false,
    });

    // 한반도 위치로 카메라 즉시 이동
    viewer.camera.setView({
      destination: Cesium.Cartesian3.fromDegrees(127.5, 38.0, 800000),
      orientation: {
        heading: Cesium.Math.toRadians(0),
        pitch: Cesium.Math.toRadians(-60),
        roll: 0
      }
    });

    // 평양 마커 추가
    viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(125.7625, 39.0392, 0),
      point: {
        pixelSize: 12,
        color: Cesium.Color.RED,
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2
      },
      label: {
        text: '평양',
        font: '14px sans-serif',
        fillColor: Cesium.Color.WHITE,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        outlineWidth: 2,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -15)
      }
    });

    // 서울 마커 추가
    viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(126.9780, 37.5665, 0),
      point: {
        pixelSize: 12,
        color: Cesium.Color.BLUE,
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2
      },
      label: {
        text: '서울',
        font: '14px sans-serif',
        fillColor: Cesium.Color.WHITE,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        outlineWidth: 2,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -15)
      }
    });

    viewerRef.current = viewer;

    return () => {
      if (viewerRef.current) {
        viewerRef.current.destroy();
      }
    };
  }, []);

  // 미사일 궤적 업데이트
  useEffect(() => {
    if (!viewerRef.current || !simulationResults || simulationResults.x.length === 0) return;

    const viewer = viewerRef.current;
    const frameLimit = Math.min(currentFrame, simulationResults.x.length);

    if (frameLimit === 0) return;

    // 궤적 좌표 계산 (km를 위도/경도로 변환)
    const positions = [];
    for (let i = 0; i < frameLimit; i++) {
      // X는 동쪽, Y는 북쪽 방향 (km 단위)
      const lonOffset = simulationResults.x[i] / 111; // 대략적인 km -> 도 변환
      const latOffset = simulationResults.y[i] / 111;
      const altitude = simulationResults.h[i] * 1000; // km -> m

      const lon = PYONGYANG.lon + lonOffset;
      const lat = PYONGYANG.lat + latOffset;

      positions.push(Cesium.Cartesian3.fromDegrees(lon, lat, altitude));
    }

    // 기존 궤적 제거
    if (trajectoryEntityRef.current) {
      viewer.entities.remove(trajectoryEntityRef.current);
    }

    // 새 궤적 추가
    trajectoryEntityRef.current = viewer.entities.add({
      polyline: {
        positions: positions,
        width: 5,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.3,
          color: Cesium.Color.RED
        }),
        clampToGround: false
      }
    });

    // 미사일 현재 위치
    if (missileEntityRef.current) {
      viewer.entities.remove(missileEntityRef.current);
    }

    if (positions.length > 0) {
      const lastPos = positions[positions.length - 1];
      missileEntityRef.current = viewer.entities.add({
        position: lastPos,
        point: {
          pixelSize: 15,
          color: Cesium.Color.RED,
          outlineColor: Cesium.Color.WHITE,
          outlineWidth: 2
        },
        label: {
          text: '미사일',
          font: '14px sans-serif',
          fillColor: Cesium.Color.WHITE,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          outlineWidth: 2,
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          pixelOffset: new Cesium.Cartesian2(0, -20)
        }
      });
    }

  }, [simulationResults, currentFrame]);

  return (
    <div 
      ref={cesiumContainerRef} 
      style={{ 
        width: '100%', 
        height: '500px',
        borderRadius: '8px',
        overflow: 'hidden'
      }} 
    />
  );
};

export default CesiumView;
