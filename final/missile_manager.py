#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Missile Manager - Integrated Missile Identification System
===========================================================

Handles:
1. Random missile selection (SCUD-B, Nodong, KN-23)
2. Real-time trajectory simulation
3. Signature-based identification
4. API server for frontend communication

Author: Data Science Project
Date: 2024
"""

import numpy as np
import json
import random
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

# Import simulators
from missile_6dof_true import True6DOFSimulator, MISSILES
from kn23_depressed import KN23Depressed
from config_6dof import MISSILE_TYPES


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class MissileSignature:
    """Missile signature profile for identification"""
    missile_type: str
    max_altitude_km: float
    range_km: float
    flight_time_s: float
    max_velocity_ms: float
    has_pullup: bool
    trajectory_type: str  # 'ballistic', 'quasi-ballistic', 'depressed'
    

@dataclass
class IdentificationResult:
    """Result of missile identification"""
    predicted_type: str
    confidence: float
    reasons: List[str]
    features: Dict[str, float]
    timestamp: float


# =============================================================================
# Signature Profiles (Reference Data)
# =============================================================================

SIGNATURE_PROFILES = {
    "SCUD-B": {
        "altitude_range": (80, 120),      # km
        "range_range": (250, 350),        # km
        "trajectory_type": "ballistic",
        "has_pullup": False,
        "burn_time": 65,
        "description": "Standard ballistic trajectory, short range"
    },
    "Nodong": {
        "altitude_range": (250, 400),     # km
        "range_range": (1000, 1500),      # km
        "trajectory_type": "ballistic",
        "has_pullup": False,
        "burn_time": 95,
        "description": "High parabolic arc, medium-long range"
    },
    "KN-23": {
        "altitude_range": (40, 70),       # km (depressed trajectory)
        "range_range": (400, 800),        # km
        "trajectory_type": "quasi-ballistic",
        "has_pullup": True,
        "burn_time": 78,
        "description": "Low altitude, pull-up maneuver in terminal phase"
    }
}


# =============================================================================
# Missile Manager Class
# =============================================================================

class MissileManager:
    """
    Manages missile simulation and identification
    
    Features:
    - Random missile selection (hidden from UI)
    - Real-time trajectory streaming
    - Signature-based identification
    """
    
    def __init__(self):
        self.current_missile: Optional[str] = None
        self.simulator = None
        self.trajectory_buffer: List[Dict] = []
        self.is_simulating = False
        self.simulation_thread = None
        self.identification_result: Optional[IdentificationResult] = None
        
        # Callback for real-time data
        self.on_trajectory_update = None
        self.on_identification_complete = None
        
        print("✓ MissileManager initialized")
        print(f"  Available missiles: {list(SIGNATURE_PROFILES.keys())}")
    
    def select_random_missile(self) -> str:
        """Randomly select a missile type (hidden from UI)"""
        self.current_missile = random.choice(list(SIGNATURE_PROFILES.keys()))
        print(f"\n[HIDDEN] Selected missile: {self.current_missile}")
        return "UNKNOWN"  # Don't reveal to UI
    
    def get_simulator(self, missile_type: str):
        """Get appropriate simulator for missile type"""
        if missile_type == "KN-23":
            return KN23Depressed()
        else:
            return True6DOFSimulator(missile_type=missile_type)
    
    def run_simulation(self, launch_angle: float = 45, azimuth: float = 90) -> Dict:
        """
        Run simulation and return trajectory data
        
        Returns trajectory without revealing missile type
        """
        if self.current_missile is None:
            self.select_random_missile()
        
        print(f"\n{'='*60}")
        print(f"Starting simulation (Missile: HIDDEN)")
        print(f"Launch angle: {launch_angle}°, Azimuth: {azimuth}°")
        print(f"{'='*60}")
        
        self.trajectory_buffer = []
        self.is_simulating = True
        
        # Get simulator
        sim = self.get_simulator(self.current_missile)
        
        # Run simulation
        if self.current_missile == "KN-23":
            result = sim.simulate(launch_angle=launch_angle)
            trajectory = self._convert_kn23_result(result)
        else:
            result = sim.simulate(elevation_deg=launch_angle, azimuth_deg=azimuth)
            trajectory = self._convert_6dof_result(result)
        
        self.trajectory_buffer = trajectory
        self.is_simulating = False
        
        # Run identification
        self.identification_result = self.identify_missile(trajectory)
        
        return {
            "trajectory": trajectory,
            "identification": asdict(self.identification_result) if self.identification_result else None,
            "actual_type": None  # Hidden until user guesses
        }
    
    def _convert_kn23_result(self, result: Dict) -> List[Dict]:
        """Convert KN23Depressed result to standard format"""
        trajectory = []
        t = result['time']
        x = result['x']
        z = result['z']
        V = result['V']
        theta = result['theta']
        gamma = result['gamma']
        alpha = result['alpha']
        
        for i in range(len(t)):
            trajectory.append({
                "time": float(t[i]),
                "x": float(x[i]),
                "y": 0.0,  # 2D simulation
                "z": float(z[i]),
                "velocity": float(V[i]),
                "theta": float(theta[i]),
                "gamma": float(gamma[i]),
                "alpha": float(alpha[i])
            })
        
        return trajectory
    
    def _convert_6dof_result(self, result: Dict) -> List[Dict]:
        """Convert True6DOF result to standard format"""
        trajectory = []
        t = result['time']
        x = result['x']
        y = result['y']
        z = result['z']
        V = result['V']
        theta = result['theta']
        phi = result['phi']
        psi = result['psi']
        
        for i in range(len(t)):
            trajectory.append({
                "time": float(t[i]),
                "x": float(x[i]),
                "y": float(y[i]),
                "z": float(z[i]),
                "velocity": float(V[i]),
                "theta": float(theta[i]),
                "gamma": float(np.arctan2(result['w'][i], result['u'][i])) if 'w' in result else 0.0,
                "alpha": float(np.arctan2(result['w'][i], max(result['u'][i], 0.1))) if 'w' in result else 0.0
            })
        
        return trajectory
    
    def identify_missile(self, trajectory: List[Dict]) -> IdentificationResult:
        """
        Identify missile type based on trajectory features
        
        Features analyzed:
        - Peak altitude
        - Range
        - Pull-up maneuver detection
        - Trajectory shape
        """
        if not trajectory:
            return IdentificationResult(
                predicted_type="UNKNOWN",
                confidence=0.0,
                reasons=["No trajectory data"],
                features={},
                timestamp=time.time()
            )
        
        # Extract features
        features = self._extract_features(trajectory)
        
        # Score each missile type
        scores = {}
        reasons = {}
        
        for missile_type, profile in SIGNATURE_PROFILES.items():
            score = 0.0
            type_reasons = []
            
            # Feature A: Altitude check
            alt_min, alt_max = profile["altitude_range"]
            if alt_min <= features["max_altitude_km"] <= alt_max:
                score += 30
                type_reasons.append(f"Altitude {features['max_altitude_km']:.1f}km matches {missile_type} profile ({alt_min}-{alt_max}km)")
            elif features["max_altitude_km"] < alt_min:
                # Low altitude - likely KN-23
                if missile_type == "KN-23":
                    score += 25
                    type_reasons.append(f"Low altitude ({features['max_altitude_km']:.1f}km) suggests depressed trajectory")
            
            # Feature B: Range check
            range_min, range_max = profile["range_range"]
            if range_min <= features["range_km"] <= range_max:
                score += 25
                type_reasons.append(f"Range {features['range_km']:.1f}km matches {missile_type} profile")
            
            # Feature C: Pull-up detection (critical for KN-23)
            if profile["has_pullup"] and features["has_pullup"]:
                score += 30
                type_reasons.append("Terminal pull-up maneuver detected")
            elif not profile["has_pullup"] and not features["has_pullup"]:
                score += 15
                type_reasons.append("No pull-up maneuver (standard ballistic)")
            
            # Feature D: Trajectory type
            if features["max_altitude_km"] < 70 and features["range_km"] > 400:
                # Quasi-ballistic signature
                if missile_type == "KN-23":
                    score += 20
                    type_reasons.append("Quasi-ballistic trajectory pattern")
            elif features["max_altitude_km"] > 200:
                # High arc - likely Nodong
                if missile_type == "Nodong":
                    score += 20
                    type_reasons.append("High parabolic arc detected")
            
            scores[missile_type] = score
            reasons[missile_type] = type_reasons
        
        # Find best match
        best_type = max(scores, key=scores.get)
        total_score = sum(scores.values())
        confidence = scores[best_type] / max(total_score, 1) * 100
        
        # Adjust confidence based on feature clarity
        if features["max_altitude_km"] < 70 and features["has_pullup"]:
            confidence = min(confidence + 15, 99)  # Strong KN-23 indicator
        
        return IdentificationResult(
            predicted_type=best_type,
            confidence=round(confidence, 1),
            reasons=reasons[best_type],
            features=features,
            timestamp=time.time()
        )
    
    def _extract_features(self, trajectory: List[Dict]) -> Dict[str, float]:
        """Extract identification features from trajectory"""
        if not trajectory:
            return {}
        
        # Basic features
        altitudes = [p["z"] for p in trajectory]
        times = [p["time"] for p in trajectory]
        
        max_altitude = max(altitudes)
        max_altitude_km = max_altitude / 1000
        
        # Range calculation
        final_x = trajectory[-1]["x"]
        final_y = trajectory[-1]["y"]
        range_km = np.sqrt(final_x**2 + final_y**2) / 1000
        
        # Flight time
        flight_time = times[-1]
        
        # Max velocity
        velocities = [p["velocity"] for p in trajectory]
        max_velocity = max(velocities)
        
        # Pull-up detection
        has_pullup = self._detect_pullup(trajectory)
        
        # Apogee timing (early apogee = depressed trajectory)
        apogee_idx = altitudes.index(max_altitude)
        apogee_time_ratio = times[apogee_idx] / flight_time
        
        return {
            "max_altitude_km": round(max_altitude_km, 1),
            "range_km": round(range_km, 1),
            "flight_time_s": round(flight_time, 1),
            "max_velocity_ms": round(max_velocity, 1),
            "has_pullup": has_pullup,
            "apogee_time_ratio": round(apogee_time_ratio, 2)
        }
    
    def _detect_pullup(self, trajectory: List[Dict]) -> bool:
        """
        Detect pull-up maneuver in terminal phase
        
        KN-23 signature: sudden pitch-up at 15-25km altitude during descent
        """
        if len(trajectory) < 50:
            return False
        
        # Find descent phase (after apogee)
        altitudes = [p["z"] for p in trajectory]
        apogee_idx = altitudes.index(max(altitudes))
        
        # Look at terminal phase (last 30% of descent)
        descent_start = apogee_idx
        descent_end = len(trajectory) - 1
        terminal_start = descent_start + int((descent_end - descent_start) * 0.5)
        
        # Check for pitch rate changes in terminal phase
        for i in range(terminal_start, min(descent_end, len(trajectory) - 5)):
            alt_km = trajectory[i]["z"] / 1000
            
            # Look for pull-up in 10-30km altitude range
            if 10 < alt_km < 30:
                # Check gamma (flight path angle) change
                if i > 0:
                    gamma_prev = trajectory[i-1].get("gamma", 0)
                    gamma_curr = trajectory[i].get("gamma", 0)
                    gamma_change = gamma_curr - gamma_prev
                    
                    # Significant positive gamma change = pull-up
                    if gamma_change > 0.02:  # ~1 degree
                        return True
                
                # Check alpha (angle of attack) spike
                alpha = abs(trajectory[i].get("alpha", 0))
                if alpha > 0.15:  # ~8.5 degrees
                    return True
        
        return False
    
    def reveal_actual_type(self) -> str:
        """Reveal the actual missile type (after user guess)"""
        return self.current_missile
    
    def get_trajectory_at_time(self, t: float) -> Optional[Dict]:
        """Get trajectory point at specific time"""
        if not self.trajectory_buffer:
            return None
        
        for point in self.trajectory_buffer:
            if point["time"] >= t:
                return point
        
        return self.trajectory_buffer[-1]
    
    def reset(self):
        """Reset manager for new simulation"""
        self.current_missile = None
        self.trajectory_buffer = []
        self.identification_result = None
        self.is_simulating = False


# =============================================================================
# API Server for Frontend Communication
# =============================================================================

class MissileAPIHandler(BaseHTTPRequestHandler):
    """HTTP request handler for missile API"""
    
    manager = MissileManager()
    
    def do_GET(self):
        """Handle GET requests"""
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)
        
        response = {"error": "Unknown endpoint"}
        status = 404
        
        if path == "/api/status":
            response = {
                "status": "ready",
                "is_simulating": self.manager.is_simulating,
                "has_trajectory": len(self.manager.trajectory_buffer) > 0
            }
            status = 200
            
        elif path == "/api/trajectory":
            if self.manager.trajectory_buffer:
                # Return trajectory (sampled for performance)
                step = max(1, len(self.manager.trajectory_buffer) // 100)
                response = {
                    "trajectory": self.manager.trajectory_buffer[::step],
                    "total_points": len(self.manager.trajectory_buffer)
                }
                status = 200
            else:
                response = {"error": "No trajectory data"}
                status = 404
                
        elif path == "/api/identification":
            if self.manager.identification_result:
                response = asdict(self.manager.identification_result)
                status = 200
            else:
                response = {"error": "No identification result"}
                status = 404
                
        elif path == "/api/reveal":
            actual = self.manager.reveal_actual_type()
            response = {
                "actual_type": actual,
                "profile": SIGNATURE_PROFILES.get(actual, {})
            }
            status = 200
        
        self._send_response(response, status)
    
    def do_POST(self):
        """Handle POST requests"""
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        
        # Read request body
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8')
        
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            data = {}
        
        response = {"error": "Unknown endpoint"}
        status = 404
        
        if path == "/api/launch":
            # Start new simulation
            launch_angle = data.get("launch_angle", 45)
            azimuth = data.get("azimuth", 90)
            
            # Select random missile
            self.manager.select_random_missile()
            
            # Run simulation
            result = self.manager.run_simulation(launch_angle, azimuth)
            
            response = {
                "status": "completed",
                "trajectory_points": len(result["trajectory"]),
                "identification": result["identification"]
            }
            status = 200
            
        elif path == "/api/reset":
            self.manager.reset()
            response = {"status": "reset"}
            status = 200
        
        self._send_response(response, status)
    
    def _send_response(self, data: Dict, status: int = 200):
        """Send JSON response"""
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))
    
    def do_OPTIONS(self):
        """Handle CORS preflight"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        """Suppress default logging"""
        pass


def run_api_server(port: int = 5000):
    """Run the API server"""
    server = HTTPServer(('localhost', port), MissileAPIHandler)
    print(f"\n{'='*60}")
    print(f"🚀 Missile API Server running on http://localhost:{port}")
    print(f"{'='*60}")
    print(f"\nEndpoints:")
    print(f"  GET  /api/status         - Server status")
    print(f"  POST /api/launch         - Start simulation")
    print(f"  GET  /api/trajectory     - Get trajectory data")
    print(f"  GET  /api/identification - Get identification result")
    print(f"  GET  /api/reveal         - Reveal actual missile type")
    print(f"  POST /api/reset          - Reset for new simulation")
    print(f"\nPress Ctrl+C to stop")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nServer stopped.")
        server.shutdown()


# =============================================================================
# File-based Communication (Alternative to API)
# =============================================================================

def save_trajectory_for_frontend(trajectory: List[Dict], output_path: str = None):
    """Save trajectory data as JSON for frontend consumption"""
    if output_path is None:
        output_path = Path(__file__).parent.parent / "lastpang" / "public" / "trajectory_data.json"
    
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(trajectory, f, ensure_ascii=False, indent=2)
    
    print(f"✓ Trajectory saved: {output_path}")
    return str(output_path)


def save_identification_for_frontend(result: IdentificationResult, output_path: str = None):
    """Save identification result as JSON for frontend"""
    if output_path is None:
        output_path = Path(__file__).parent.parent / "lastpang" / "public" / "identification_result.json"
    
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(asdict(result), f, ensure_ascii=False, indent=2)
    
    print(f"✓ Identification saved: {output_path}")
    return str(output_path)


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    """Main function - demonstrates the system"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Missile Manager - Identification System')
    parser.add_argument('--server', action='store_true', help='Run API server')
    parser.add_argument('--port', type=int, default=5000, help='API server port')
    parser.add_argument('--demo', action='store_true', help='Run demo simulation')
    parser.add_argument('--angle', type=float, default=45, help='Launch angle (degrees)')
    
    args = parser.parse_args()
    
    if args.server:
        run_api_server(args.port)
    elif args.demo:
        # Demo mode
        print("\n" + "="*60)
        print("🎯 Missile Identification System - Demo")
        print("="*60)
        
        manager = MissileManager()
        
        # Random selection (hidden)
        manager.select_random_missile()
        
        # Run simulation
        result = manager.run_simulation(launch_angle=args.angle)
        
        # Show identification result
        ident = result["identification"]
        print(f"\n{'='*60}")
        print("📊 IDENTIFICATION RESULT")
        print(f"{'='*60}")
        print(f"  Predicted Type: {ident['predicted_type']}")
        print(f"  Confidence: {ident['confidence']}%")
        print(f"\n  Reasons:")
        for reason in ident['reasons']:
            print(f"    • {reason}")
        print(f"\n  Features:")
        for key, value in ident['features'].items():
            print(f"    • {key}: {value}")
        
        # Reveal actual type
        actual = manager.reveal_actual_type()
        print(f"\n{'='*60}")
        print(f"🎯 ACTUAL MISSILE: {actual}")
        print(f"{'='*60}")
        
        correct = ident['predicted_type'] == actual
        print(f"  Prediction: {'✓ CORRECT' if correct else '✗ INCORRECT'}")
        
        # Save for frontend
        save_trajectory_for_frontend(result["trajectory"])
        save_identification_for_frontend(manager.identification_result)
        
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
