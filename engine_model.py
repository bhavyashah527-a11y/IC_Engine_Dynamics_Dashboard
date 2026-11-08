import numpy as np
import pandas as pd

def slider_crank_kinematics(r, l, rpm, samples=721):
    """Exact slider-crank kinematics. r,l in m; output spans 0–360 degrees."""
    if r <= 0 or l <= r:
        raise ValueError("Require r > 0 and connecting rod length l > r.")
    if rpm < 0:
        raise ValueError("RPM cannot be negative.")
    theta = np.linspace(0, 2*np.pi, samples)
    omega = 2*np.pi*rpm/60
    q = np.sqrt(l*l - (r*np.sin(theta))**2)
    x = r*(1-np.cos(theta)) + l - q
    dx = r*np.sin(theta) + r*r*np.sin(theta)*np.cos(theta)/q
    ddx = (r*np.cos(theta)
           + r*r*(np.cos(theta)**2-np.sin(theta)**2)/q
           + r**4*(np.sin(theta)*np.cos(theta))**2/q**3)
    ratio = (r/l)*np.sin(theta)
    beta = np.arcsin(ratio)
    beta_dot = omega*(r/l)*np.cos(theta)/np.sqrt(1-ratio**2)
    beta_ddot = np.gradient(beta_dot, theta)*omega
    return pd.DataFrame({
        "crank_angle_deg": np.degrees(theta),
        "piston_displacement_mm": x*1000,
        "piston_velocity_m_s": omega*dx,
        "piston_acceleration_m_s2": omega**2*ddx,
        "rod_angle_deg": np.degrees(beta),
        "rod_angular_velocity_rad_s": beta_dot,
        "rod_angular_acceleration_rad_s2": beta_ddot,
    })

def force_analysis(data, r, l, rpm, m_rec, m_rot, bore=0.0, pressure=0.0):
    theta = np.radians(data.crank_angle_deg.to_numpy())
    omega = 2*np.pi*rpm/60
    q = np.sqrt(l*l-(r*np.sin(theta))**2)
    dx_dtheta = r*np.sin(theta)+r*r*np.sin(theta)*np.cos(theta)/q
    gas_force = pressure*np.pi*bore**2/4
    inertia_force = -m_rec*data.piston_acceleration_m_s2.to_numpy()
    piston_effort = gas_force + inertia_force
    out = data.copy()
    out["gas_force_N"] = gas_force
    out["reciprocating_inertia_force_N"] = inertia_force
    out["piston_effort_N"] = piston_effort
    out["idealized_crank_torque_Nm"] = piston_effort*dx_dtheta
    out["primary_inertia_force_N"] = m_rec*r*omega**2*np.cos(theta)
    out["secondary_inertia_force_N"] = m_rec*r*omega**2*(r/l)*np.cos(2*theta)
    out["rotating_unbalance_x_N"] = m_rot*r*omega**2*np.cos(theta)
    out["rotating_unbalance_y_N"] = m_rot*r*omega**2*np.sin(theta)
    return out

def balancing_analysis(data, r, rpm, m_rec, m_rot, fraction, rb):
    if not 0 <= fraction <= 1:
        raise ValueError("Balancing fraction must be between 0 and 1.")
    if rb <= 0:
        raise ValueError("Counterweight radius must be positive.")
    omega = 2*np.pi*rpm/60
    theta = np.radians(data.crank_angle_deg.to_numpy())
    m_cw = (m_rot + fraction*m_rec)*r/rb
    scale = omega**2
    rec_primary = m_rec*r*scale*np.cos(theta)
    rot_x = m_rot*r*scale*np.cos(theta)
    rot_y = m_rot*r*scale*np.sin(theta)
    cw_x = -m_cw*rb*scale*np.cos(theta)
    cw_y = -m_cw*rb*scale*np.sin(theta)
    out = data.copy()
    out["primary_reciprocating_force_N"] = rec_primary
    out["rotating_force_x_N"] = rot_x
    out["rotating_force_y_N"] = rot_y
    out["counterweight_force_x_N"] = cw_x
    out["counterweight_force_y_N"] = cw_y
    out["residual_shaking_force_x_N"] = rec_primary+rot_x+cw_x
    out["residual_shaking_force_y_N"] = rot_y+cw_y
    out["secondary_reciprocating_force_N"] = (
        m_rec*r*scale*(r/(float(data.attrs.get("rod_ratio_denominator_m", 1.0))))*np.cos(2*theta)
        if data.attrs.get("rod_ratio_denominator_m") else np.zeros_like(theta)
    )
    info = {
        "counterweight_mass_kg": m_cw,
        "residual_x_rms_N": float(np.sqrt(np.mean(out.residual_shaking_force_x_N**2))),
        "residual_y_rms_N": float(np.sqrt(np.mean(out.residual_shaking_force_y_N**2))),
    }
    return out, info

def export_dataframe(data):
    return data.to_csv(index=False).encode("utf-8")
