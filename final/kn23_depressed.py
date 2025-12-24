#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KN-23 Depressed Trajectory Simulator
=====================================

KN-23 (Hwasong-11Ga) 미사일의 편평 탄도(Depressed Trajectory) 시뮬레이션

목표:
- Max Altitude: 40-60km (일반 탄도 미사일 대비 매우 낮음)
- Range: 600-800km
- Pull-up 기동: 종말 단계에서 급격한 피치업
- Max Alpha: < 15도
"""

import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path

# Physical Constants
G0 = 9.80665
R_EARTH = 6371000
RHO_SL = 1.225
T_SL = 288.15
P_SL = 101325
R_AIR = 287.05
GAMMA_AIR = 1.4


def get_atmosphere(h):
    """ISA 1976 Standard Atmosphere"""
    h = max(0, h)
    if h <= 11000:
        T = T_SL - 0.0065 * h
        P = P_SL * (T / T_SL) ** 5.2561
    elif h <= 25000:
        T = 216.65
        P = 22632.1 * np.exp(-0.00015768 * (h - 11000))
    else:
        T = 216.65 + 0.003 * (h - 25000)
        P = 2488.7 * (T / 216.65) ** (-11.388)
    rho = P / (R_AIR * T)
    a = np.sqrt(GAMMA_AIR * R_AIR * T)
    return rho, P, T, a


def get_cd(mach, alpha_deg=0):
    """Drag coefficient"""
    if mach < 0.8:
        cd = 0.20
    elif mach < 1.2:
        cd = 0.20 + 0.4 * (mach - 0.8) / 0.4
    elif mach < 2.0:
        cd = 0.60 - 0.2 * (mach - 1.2) / 0.8
    else:
        cd = 0.40
    return cd + 0.01 * (alpha_deg / 10)**2


class KN23Depressed:
    """
    KN-23 Depressed Trajectory Simulator
    
    3DOF Point Mass with pitch control
    State: [x, z, Vx, Vz, theta]
    """
    
    def __init__(self):
        # Missile parameters
        self.mass_total = 3750
        self.mass_prop = 2950
        self.mass_dry = 800
        self.burn_time = 78
        self.isp_sea = 295
        self.isp_vac = 315
        self.S_ref = np.pi * (0.95/2)**2
        self.CL_alpha = 4.0
        
        # State tracking
        self._max_alt = 0
        self._descending = False
        self._pullup_triggered = False
        
        print("✓ KN-23 Depressed Trajectory Simulator")
        print(f"  Mass: {self.mass_total} kg, Burn: {self.burn_time} s")
    
    def get_mass(self, t):
        if t < self.burn_time:
            return self.mass_dry + self.mass_prop * (1 - t/self.burn_time)
        return self.mass_dry
    
    def get_thrust(self, t, h):
        if t >= self.burn_time:
            return 0.0
        _, P, _, _ = get_atmosphere(h)
        isp = self.isp_sea + (self.isp_vac - self.isp_sea) * (1 - P/P_SL)
        return isp * (self.mass_prop / self.burn_time) * G0
    
    def dynamics(self, t, state):
        x, z, Vx, Vz, theta = state
        z = max(z, 0)
        V = max(np.sqrt(Vx**2 + Vz**2), 1.0)
        gamma = np.arctan2(Vz, max(Vx, 1.0))
        alpha = np.clip(theta - gamma, -15*np.pi/180, 15*np.pi/180)
        
        rho, _, _, a = get_atmosphere(z)
        mach = V / a
        q_dyn = 0.5 * rho * V**2
        
        m = self.get_mass(t)
        thrust = self.get_thrust(t, z)
        
        CD = get_cd(mach, abs(alpha)*180/np.pi)
        CL = self.CL_alpha * alpha
        D = q_dyn * self.S_ref * CD
        L = q_dyn * self.S_ref * CL
        
        g = G0 * (R_EARTH / (R_EARTH + z))**2
        
        # Forces
        Fx = thrust * np.cos(theta) - D * Vx/V - L * np.sin(gamma)
        Fz = thrust * np.sin(theta) - D * Vz/V + L * np.cos(gamma) - m * g
        
        ax = Fx / m
        az = Fz / m
        
        # Pitch control
        dtheta = self._pitch_control(t, z, Vz, theta, gamma, alpha)
        
        return np.array([Vx, Vz, ax, az, dtheta])
    
    def _pitch_control(self, t, z, Vz, theta, gamma, alpha):
        """
        Pitch control for KN-23 Quasi-Ballistic Trajectory
        
        Key features:
        1. Boost: Rapid pitch-over to 23 deg
        2. Cruise: "Flat Arc" at 40-60km (maintain gamma ≈ 0)
        3. Descent: Follow velocity vector
        4. Pull-up: Visible "bump" - sudden pitch-up at 15-25km
        5. Terminal: Final dive
        """
        # Update state
        if z > self._max_alt:
            self._max_alt = z
        elif z < self._max_alt - 500:
            self._descending = True
        
        # Hard limit: Max altitude 60km
        if z > 60000 and not self._descending:
            # Force pitch down if exceeding 60km
            dtheta = -5.0 * np.pi / 180
            return np.clip(dtheta, -15*np.pi/180, 15*np.pi/180)
        
        # Determine phase
        if t < self.burn_time:
            phase = "Boost"
        elif not self._descending and z >= 40000:
            phase = "Cruise"  # Flat arc at 40-60km
        elif not self._descending:
            phase = "Climb"
        elif z > 25000:
            phase = "Descent"
        elif z > 12000:
            phase = "Pull-up"  # Terminal pull-up bump
        else:
            phase = "Terminal"
        
        # Control law
        if phase == "Boost":
            # Rapid pitch-over to 22.3 deg (depressed trajectory for 40-60km apogee, 600-800km range)
            target = 22.3 * np.pi / 180
            dtheta = 2.5 * (target - theta)
            
        elif phase == "Climb":
            # Continue climbing, follow velocity
            dtheta = -2.0 * alpha
            
        elif phase == "Cruise":
            # ============================================================
            # "Flat Arc": Maintain horizontal flight (gamma ≈ 0)
            # This creates the flattened mid-section at 40-60km
            # ============================================================
            target_gamma = 0.0  # Horizontal flight
            gamma_error = target_gamma - gamma
            
            # Adjust pitch to maintain horizontal flight
            dtheta = 3.0 * gamma_error
            
            # Limit alpha to prevent stall
            if abs(alpha) > 12 * np.pi / 180:
                dtheta = -4.0 * alpha
            
        elif phase == "Descent":
            # Follow velocity vector during descent
            dtheta = -3.0 * alpha
            
        elif phase == "Pull-up":
            # ============================================================
            # Terminal Pull-up "Bump": Sudden pitch-up for evasion
            # Creates visible S-curve in trajectory
            # ============================================================
            target_alpha = 12 * np.pi / 180  # 12 deg AoA
            alpha_error = target_alpha - alpha
            
            # Strong pitch-up command
            dtheta = 5.0 * alpha_error
            self._pullup_triggered = True
            
            # Limit alpha to 15 deg
            if alpha > 14 * np.pi / 180:
                dtheta = -3.0 * (alpha - 12 * np.pi / 180)
            
        else:  # Terminal
            # Final dive - follow velocity vector
            dtheta = -6.0 * alpha
        
        # Rate limit (15 deg/s max)
        return np.clip(dtheta, -15*np.pi/180, 15*np.pi/180)
    
    def ground_event(self, t, state):
        if t < 10:
            return 1000
        return state[1]
    ground_event.terminal = True
    ground_event.direction = -1
    
    def simulate(self, launch_angle=20):
        print(f"\n{'='*60}")
        print(f"KN-23 Depressed Trajectory Simulation")
        print(f"  Launch angle: {launch_angle}°")
        print(f"{'='*60}")
        
        theta0 = launch_angle * np.pi / 180
        V0 = 50.0
        
        state0 = np.array([0, 0, V0*np.cos(theta0), V0*np.sin(theta0), theta0])
        
        self._max_alt = 0
        self._descending = False
        self._pullup_triggered = False
        
        sol = solve_ivp(
            self.dynamics, [0, 600], state0,
            method='RK45', events=[self.ground_event],
            max_step=1.0, rtol=1e-5, atol=1e-7
        )
        
        t = sol.t
        x, z = sol.y[0], sol.y[1]
        Vx, Vz = sol.y[2], sol.y[3]
        theta = sol.y[4]
        
        V = np.sqrt(Vx**2 + Vz**2)
        gamma = np.arctan2(Vz, np.maximum(Vx, 1.0))
        alpha = theta - gamma
        
        range_km = x[-1] / 1000
        max_alt_km = np.max(z) / 1000
        max_alpha = np.max(np.abs(alpha)) * 180 / np.pi
        
        print(f"\nResults:")
        print(f"  Range: {range_km:.1f} km")
        print(f"  Max altitude: {max_alt_km:.1f} km")
        print(f"  Flight time: {t[-1]:.1f} s")
        print(f"  Max |alpha|: {max_alpha:.1f}°")
        print(f"  Pull-up triggered: {self._pullup_triggered}")
        print(f"  Reference: 690 km, Error: {(range_km-690)/690*100:+.1f}%")
        
        return {
            'time': t, 'x': x, 'z': z,
            'Vx': Vx, 'Vz': Vz, 'V': V,
            'theta': theta, 'gamma': gamma, 'alpha': alpha,
            'range_km': range_km, 'max_alt_km': max_alt_km,
            'max_alpha_deg': max_alpha
        }


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    sim = KN23Depressed()
    result = sim.simulate(launch_angle=20)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Trajectory
    axes[0,0].plot(result['x']/1000, result['z']/1000, 'g-', lw=2)
    axes[0,0].axhline(y=50, color='r', ls='--', label='Target (50km)')
    axes[0,0].axhline(y=40, color='orange', ls='--', alpha=0.5)
    axes[0,0].axhline(y=60, color='orange', ls='--', alpha=0.5)
    axes[0,0].set_xlabel('Range (km)')
    axes[0,0].set_ylabel('Altitude (km)')
    axes[0,0].set_title('KN-23 Depressed Trajectory')
    axes[0,0].legend()
    axes[0,0].grid(True)
    
    # Velocity
    axes[0,1].plot(result['time'], result['V'], 'b-', lw=2)
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Velocity (m/s)')
    axes[0,1].set_title('Velocity')
    axes[0,1].grid(True)
    
    # Angles
    axes[1,0].plot(result['time'], result['theta']*180/np.pi, 'b-', label='Pitch')
    axes[1,0].plot(result['time'], result['gamma']*180/np.pi, 'r--', label='Flight path')
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Angle (deg)')
    axes[1,0].set_title('Pitch & Flight Path')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    # Alpha
    axes[1,1].plot(result['time'], result['alpha']*180/np.pi, 'g-', lw=2)
    axes[1,1].axhline(y=15, color='r', ls='--', label='Limit')
    axes[1,1].axhline(y=-15, color='r', ls='--')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Alpha (deg)')
    axes[1,1].set_title('Angle of Attack')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    plt.tight_layout()
    
    out_dir = Path(__file__).parent / "results_6dof"
    out_dir.mkdir(exist_ok=True)
    plt.savefig(out_dir / "kn23_depressed.png", dpi=150)
    print(f"\n✓ Plot saved: {out_dir / 'kn23_depressed.png'}")
    plt.show()