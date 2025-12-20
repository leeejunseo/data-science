import React, { useState, useEffect, useCallback } from 'react';
import { Target, Radar, AlertTriangle, CheckCircle, XCircle, RefreshCw } from 'lucide-react';

/**
 * MissileIdentifier Component
 * 
 * Connects to the Python backend (missile_manager.py) for:
 * - Real-time trajectory data
 * - Signature-based identification
 * - Game-like identification challenge
 */

const API_BASE = 'http://localhost:5000';

const MissileIdentifier = ({ trajectoryData, onIdentificationComplete }) => {
  const [identification, setIdentification] = useState(null);
  const [actualType, setActualType] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [userGuess, setUserGuess] = useState(null);
  const [showResult, setShowResult] = useState(false);
  const [features, setFeatures] = useState(null);

  // Fetch identification from backend
  const fetchIdentification = useCallback(async () => {
    try {
      const response = await fetch(`${API_BASE}/api/identification`);
      if (response.ok) {
        const data = await response.json();
        setIdentification(data);
        setFeatures(data.features);
        if (onIdentificationComplete) {
          onIdentificationComplete(data);
        }
      }
    } catch (error) {
      console.log('Backend not available, using local analysis');
      // Fallback: analyze trajectory locally
      if (trajectoryData && trajectoryData.length > 0) {
        const localResult = analyzeTrajectoryLocally(trajectoryData);
        setIdentification(localResult);
        setFeatures(localResult.features);
      }
    }
  }, [trajectoryData, onIdentificationComplete]);

  // Reveal actual missile type
  const revealActual = async () => {
    try {
      const response = await fetch(`${API_BASE}/api/reveal`);
      if (response.ok) {
        const data = await response.json();
        setActualType(data.actual_type);
        setShowResult(true);
      }
    } catch (error) {
      console.log('Could not reveal - backend not available');
    }
  };

  // Local trajectory analysis (fallback when backend unavailable)
  const analyzeTrajectoryLocally = (trajectory) => {
    if (!trajectory || trajectory.length === 0) {
      return null;
    }

    // Extract features
    const altitudes = trajectory.map(p => p.h || p.z || 0);
    const maxAltitude = Math.max(...altitudes);
    const maxAltitudeKm = maxAltitude / 1000;

    const finalX = trajectory[trajectory.length - 1].x || 0;
    const finalY = trajectory[trajectory.length - 1].y || 0;
    const rangeKm = Math.sqrt(finalX * finalX + finalY * finalY) / 1000;

    const flightTime = trajectory[trajectory.length - 1].time || 0;

    // Detect pull-up (simplified)
    let hasPullup = false;
    const apogeeIdx = altitudes.indexOf(maxAltitude);
    if (apogeeIdx < trajectory.length - 10) {
      // Check terminal phase for gamma changes
      for (let i = Math.floor(trajectory.length * 0.7); i < trajectory.length - 1; i++) {
        const alt = altitudes[i] / 1000;
        if (alt > 10 && alt < 30) {
          const gamma1 = trajectory[i].gamma || 0;
          const gamma2 = trajectory[i + 1].gamma || 0;
          if (gamma2 - gamma1 > 0.02) {
            hasPullup = true;
            break;
          }
        }
      }
    }

    // Identification logic
    let predictedType = 'UNKNOWN';
    let confidence = 0;
    const reasons = [];

    if (maxAltitudeKm < 70 && rangeKm > 400) {
      predictedType = 'KN-23';
      confidence = 75;
      reasons.push(`Low apogee (${maxAltitudeKm.toFixed(1)}km) suggests depressed trajectory`);
      if (hasPullup) {
        confidence += 20;
        reasons.push('Terminal pull-up maneuver detected');
      }
    } else if (maxAltitudeKm > 200) {
      predictedType = 'Nodong';
      confidence = 80;
      reasons.push(`High parabolic arc (${maxAltitudeKm.toFixed(1)}km apogee)`);
      if (rangeKm > 800) {
        confidence += 10;
        reasons.push(`Long range (${rangeKm.toFixed(1)}km) matches Nodong profile`);
      }
    } else if (maxAltitudeKm >= 70 && maxAltitudeKm <= 150) {
      predictedType = 'SCUD-B';
      confidence = 70;
      reasons.push(`Medium altitude (${maxAltitudeKm.toFixed(1)}km) matches SCUD-B profile`);
      if (rangeKm < 400) {
        confidence += 15;
        reasons.push(`Short range (${rangeKm.toFixed(1)}km) confirms SCUD-B`);
      }
    }

    return {
      predicted_type: predictedType,
      confidence: Math.min(confidence, 99),
      reasons: reasons,
      features: {
        max_altitude_km: maxAltitudeKm.toFixed(1),
        range_km: rangeKm.toFixed(1),
        flight_time_s: flightTime.toFixed(1),
        has_pullup: hasPullup
      }
    };
  };

  // Auto-analyze when trajectory data changes
  useEffect(() => {
    if (trajectoryData && trajectoryData.length > 0) {
      fetchIdentification();
    }
  }, [trajectoryData, fetchIdentification]);

  // Handle user guess
  const handleGuess = (guess) => {
    setUserGuess(guess);
    revealActual();
  };

  // Get confidence color
  const getConfidenceColor = (confidence) => {
    if (confidence >= 80) return 'text-green-400';
    if (confidence >= 60) return 'text-yellow-400';
    return 'text-red-400';
  };

  // Get missile type color
  const getMissileColor = (type) => {
    switch (type) {
      case 'SCUD-B': return 'bg-blue-600';
      case 'Nodong': return 'bg-purple-600';
      case 'KN-23': return 'bg-red-600';
      default: return 'bg-gray-600';
    }
  };

  if (!trajectoryData || trajectoryData.length === 0) {
    return (
      <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
        <div className="flex items-center gap-2 text-gray-400">
          <Radar className="w-5 h-5 animate-pulse" />
          <span>Waiting for trajectory data...</span>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-gray-800 rounded-lg p-4 border border-gray-700 space-y-4">
      {/* Header */}
      <div className="flex items-center justify-between">
        <h3 className="text-lg font-semibold text-white flex items-center gap-2">
          <Target className="w-5 h-5 text-red-400" />
          Missile Identification
        </h3>
        {isLoading && (
          <RefreshCw className="w-4 h-4 text-blue-400 animate-spin" />
        )}
      </div>

      {/* Features Display */}
      {features && (
        <div className="bg-gray-900 rounded p-3 space-y-2">
          <h4 className="text-sm font-medium text-gray-400">Detected Features:</h4>
          <div className="grid grid-cols-2 gap-2 text-sm">
            <div className="flex justify-between">
              <span className="text-gray-500">Max Altitude:</span>
              <span className="text-white font-mono">{features.max_altitude_km} km</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-500">Range:</span>
              <span className="text-white font-mono">{features.range_km} km</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-500">Flight Time:</span>
              <span className="text-white font-mono">{features.flight_time_s} s</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-500">Pull-up:</span>
              <span className={features.has_pullup ? 'text-red-400' : 'text-gray-500'}>
                {features.has_pullup ? 'DETECTED' : 'None'}
              </span>
            </div>
          </div>
        </div>
      )}

      {/* AI Prediction */}
      {identification && !showResult && (
        <div className="bg-gradient-to-r from-blue-900/50 to-purple-900/50 rounded p-3 border border-blue-700">
          <div className="flex items-center gap-2 mb-2">
            <AlertTriangle className="w-4 h-4 text-yellow-400" />
            <span className="text-sm text-gray-300">AI Analysis:</span>
          </div>
          <div className="flex items-center justify-between">
            <span className={`text-xl font-bold ${getMissileColor(identification.predicted_type)} px-3 py-1 rounded`}>
              {identification.predicted_type}
            </span>
            <span className={`text-lg font-semibold ${getConfidenceColor(identification.confidence)}`}>
              {identification.confidence}% confidence
            </span>
          </div>
          <div className="mt-2 space-y-1">
            {identification.reasons && identification.reasons.map((reason, idx) => (
              <p key={idx} className="text-xs text-gray-400">• {reason}</p>
            ))}
          </div>
        </div>
      )}

      {/* User Guess Section */}
      {!showResult && (
        <div className="space-y-2">
          <p className="text-sm text-gray-400">What do you think it is?</p>
          <div className="flex gap-2">
            {['SCUD-B', 'Nodong', 'KN-23'].map((type) => (
              <button
                key={type}
                onClick={() => handleGuess(type)}
                className={`flex-1 py-2 px-3 rounded font-semibold transition-all
                  ${getMissileColor(type)} hover:opacity-80
                  ${userGuess === type ? 'ring-2 ring-white' : ''}`}
              >
                {type}
              </button>
            ))}
          </div>
        </div>
      )}

      {/* Result Display */}
      {showResult && actualType && (
        <div className={`rounded p-4 ${userGuess === actualType ? 'bg-green-900/50 border border-green-600' : 'bg-red-900/50 border border-red-600'}`}>
          <div className="flex items-center gap-2 mb-2">
            {userGuess === actualType ? (
              <CheckCircle className="w-6 h-6 text-green-400" />
            ) : (
              <XCircle className="w-6 h-6 text-red-400" />
            )}
            <span className="text-lg font-bold text-white">
              {userGuess === actualType ? 'CORRECT!' : 'INCORRECT'}
            </span>
          </div>
          <div className="space-y-1 text-sm">
            <p className="text-gray-300">
              Actual Missile: <span className={`font-bold ${getMissileColor(actualType)} px-2 py-0.5 rounded`}>{actualType}</span>
            </p>
            <p className="text-gray-300">
              Your Guess: <span className="font-semibold">{userGuess}</span>
            </p>
            <p className="text-gray-300">
              AI Prediction: <span className="font-semibold">{identification?.predicted_type}</span>
              {identification?.predicted_type === actualType && (
                <span className="text-green-400 ml-2">✓</span>
              )}
            </p>
          </div>
        </div>
      )}
    </div>
  );
};

export default MissileIdentifier;
