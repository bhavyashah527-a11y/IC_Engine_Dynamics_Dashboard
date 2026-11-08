import numpy as np
import plotly.graph_objects as go
import streamlit as st
from engine_model import slider_crank_kinematics, force_analysis, balancing_analysis, export_dataframe

st.set_page_config(page_title="IC Engine Dynamics", page_icon="⚙️", layout="wide")
st.markdown("""
<style>
.stApp{background:#0b1220;color:#e5edf7}
[data-testid="stSidebar"]{background:#111c2e}
[data-testid="stMetric"]{background:#142238;border:1px solid #263b57;padding:14px;border-radius:10px}
[data-testid="stMetricLabel"]{color:#9fb2ca}
[data-testid="stMetricValue"]{color:#f3f7fc}
.block-container{padding-top:1.3rem}
</style>
""", unsafe_allow_html=True)

st.title("⚙️ IC ENGINE DYNAMICS")
st.caption("Slider-crank kinematics | Dynamic force analysis | Crankshaft balancing")

st.sidebar.header("MODEL INPUTS")
st.sidebar.caption("Geometry inputs in millimetres")
rpm = st.sidebar.slider("Engine speed (RPM)", 100, 6000, 1500, 50)
crank_mm = st.sidebar.number_input("Crank radius (mm)", 5.0, 150.0, 50.0, 1.0)
rod_mm = st.sidebar.number_input("Connecting rod length (mm)", 60.0, 500.0, 200.0, 1.0)
m_rec = st.sidebar.number_input("Reciprocating mass (kg)", 0.05, 20.0, 1.20, 0.05)
m_rot = st.sidebar.number_input("Equivalent rotating mass (kg)", 0.0, 20.0, 0.40, 0.05)
bore_mm = st.sidebar.number_input("Cylinder bore (mm)", 0.0, 300.0, 80.0, 1.0)
pressure_mpa = st.sidebar.number_input("Illustrative constant cylinder pressure (MPa)", 0.0, 10.0, 0.0, 0.05)
st.sidebar.divider()
st.sidebar.subheader("BALANCING INPUTS")
balance_pct = st.sidebar.slider("Reciprocating mass balancing fraction (%)", 0, 100, 50, 5)
cw_radius_mm = st.sidebar.number_input("Counterweight radius (mm)", 10.0, 250.0, 75.0, 1.0)
phase_deg = st.sidebar.slider("Crank angle for mechanism view (degrees)", 0, 360, 30, 1)

r, l, rb = crank_mm/1000, rod_mm/1000, cw_radius_mm/1000
if l <= r:
    st.error("Connecting rod length must be greater than crank radius.")
    st.stop()

kin = slider_crank_kinematics(r, l, rpm)
kin.attrs["rod_ratio_denominator_m"] = l
dyn = force_analysis(kin, r, l, rpm, m_rec, m_rot, bore_mm/1000, pressure_mpa*1e6)
bal, bal_info = balancing_analysis(kin, r, rpm, m_rec, m_rot, balance_pct/100, rb)
theta = np.radians(bal.crank_angle_deg.to_numpy())
omega = 2*np.pi*rpm/60
bal["secondary_reciprocating_force_N"] = m_rec*r*omega**2*(r/l)*np.cos(2*theta)
bal["residual_x_with_secondary_N"] = bal.residual_shaking_force_x_N + bal.secondary_reciprocating_force_N

k1,k2,k3,k4 = st.columns(4)
k1.metric("Angular speed", f"{omega:.2f} rad/s")
k2.metric("Piston stroke", f"{2*crank_mm:.1f} mm")
k3.metric("Peak piston speed", f"{dyn.piston_velocity_m_s.abs().max():.2f} m/s")
k4.metric("Peak piston acceleration", f"{dyn.piston_acceleration_m_s2.abs().max():.0f} m/s²")

tabs = st.tabs(["01 Engine model","02 Kinematics","03 Dynamic analysis","04 Balancing","05 Comparison & data"])
def chart(df, series, title, ylabel):
    fig=go.Figure()
    for col,label in series:
        fig.add_trace(go.Scatter(x=df.crank_angle_deg,y=df[col],mode="lines",name=label,line=dict(width=2.4)))
    fig.update_layout(template="plotly_dark",title=title,height=350,
        paper_bgcolor="#111c2e",plot_bgcolor="#111c2e",
        margin=dict(l=35,r=20,t=55,b=35),xaxis_title="Crank angle (degrees)",yaxis_title=ylabel)
    fig.update_xaxes(gridcolor="#263b57"); fig.update_yaxes(gridcolor="#263b57")
    return fig

with tabs[0]:
    st.subheader("Slider-crank mechanism")
    th=np.radians(phase_deg)
    xc=r*np.cos(th); yc=r*np.sin(th)
    xp=r*np.cos(th)+np.sqrt(l*l-(r*np.sin(th))**2)
    fig=go.Figure()
    fig.add_trace(go.Scatter(x=[-r*1.4,l+r*1.3],y=[0,0],mode="lines",name="Cylinder axis",line=dict(color="#64748b",width=8)))
    fig.add_trace(go.Scatter(x=[0,xc],y=[0,yc],mode="lines+markers",name="Crank",line=dict(color="#38bdf8",width=7),marker=dict(size=11)))
    fig.add_trace(go.Scatter(x=[xc,xp],y=[yc,0],mode="lines+markers",name="Connecting rod",line=dict(color="#f59e0b",width=7),marker=dict(size=11)))
    fig.add_shape(type="rect",x0=xp-.035,x1=xp+.035,y0=-.025,y1=.025,line=dict(color="#34d399",width=2),fillcolor="#166534")
    fig.update_layout(template="plotly_dark",title=f"Mechanism at {phase_deg}°",height=470,
        paper_bgcolor="#111c2e",plot_bgcolor="#111c2e",xaxis_title="Position (m)",yaxis_title="Position (m)")
    fig.update_yaxes(scaleanchor="x",scaleratio=1)
    st.plotly_chart(fig,use_container_width=True)
    disp=np.interp(phase_deg,kin.crank_angle_deg,kin.piston_displacement_mm)
    st.info(f"Piston displacement from TDC at {phase_deg}° = {disp:.2f} mm. Change crank angle in the sidebar to inspect the mechanism.")

with tabs[1]:
    st.subheader("Piston kinematics over one crank revolution")
    a,b=st.columns(2)
    a.plotly_chart(chart(kin,[("piston_displacement_mm","Displacement")],"Piston displacement","mm"),use_container_width=True)
    b.plotly_chart(chart(kin,[("piston_velocity_m_s","Velocity")],"Piston velocity","m/s"),use_container_width=True)
    a.plotly_chart(chart(kin,[("piston_acceleration_m_s2","Acceleration")],"Piston acceleration","m/s²"),use_container_width=True)
    b.plotly_chart(chart(kin,[("rod_angle_deg","Connecting rod angle")],"Connecting-rod angular position","degrees"),use_container_width=True)
    x,y,z=st.columns(3)
    x.metric("Maximum displacement",f"{kin.piston_displacement_mm.max():.2f} mm")
    y.metric("Maximum |velocity|",f"{kin.piston_velocity_m_s.abs().max():.3f} m/s")
    z.metric("Maximum |acceleration|",f"{kin.piston_acceleration_m_s2.abs().max():.1f} m/s²")

with tabs[2]:
    st.subheader("Dynamic force analysis")
    a,b=st.columns(2)
    a.plotly_chart(chart(dyn,[("reciprocating_inertia_force_N","Inertia force")],"Reciprocating inertia force","N"),use_container_width=True)
    b.plotly_chart(chart(dyn,[("primary_inertia_force_N","Primary"),("secondary_inertia_force_N","Secondary")],"Primary and secondary components","N"),use_container_width=True)
    a.plotly_chart(chart(dyn,[("piston_effort_N","Piston effort")],"Idealized piston effort","N"),use_container_width=True)
    b.plotly_chart(chart(dyn,[("idealized_crank_torque_Nm","Crank torque")],"Idealized crankshaft torque","N m"),use_container_width=True)
    st.warning("Cylinder pressure is treated as constant over the entire revolution. This is illustrative, not a realistic combustion-pressure trace. For realistic engine torque, use pressure versus crank angle.")

with tabs[3]:
    st.subheader("Counterweight sizing and residual shaking forces")
    a,b,c=st.columns(3)
    a.metric("Estimated counterweight mass",f"{bal_info['counterweight_mass_kg']:.3f} kg")
    b.metric("Residual X RMS",f"{bal_info['residual_x_rms_N']:.1f} N")
    c.metric("Residual Y RMS",f"{bal_info['residual_y_rms_N']:.1f} N")
    st.latex(r"m_b r_b = (m_{rot}+f m_{rec})r")
    st.plotly_chart(chart(bal,[("residual_x_with_secondary_N","Residual X incl. secondary"),("residual_shaking_force_y_N","Residual Y")],"Residual shaking-force components","N"),use_container_width=True)
    st.plotly_chart(chart(bal,[("primary_reciprocating_force_N","Primary"),("secondary_reciprocating_force_N","Secondary")],"Reciprocating inertia-force harmonics","N"),use_container_width=True)
    st.info("A single rotating counterweight cannot completely balance reciprocating motion in all directions. Increasing the balancing fraction reduces the primary line-of-stroke component but introduces a transverse rotating component. The secondary component remains.")

with tabs[4]:
    st.subheader("Unbalanced vs selected balancing fraction")
    unbal,_=balancing_analysis(kin,r,rpm,m_rec,m_rot,0.0,rb)
    fig=go.Figure()
    fig.add_trace(go.Scatter(x=unbal.crank_angle_deg,y=unbal.residual_shaking_force_x_N,mode="lines",name="0% balance"))
    fig.add_trace(go.Scatter(x=bal.crank_angle_deg,y=bal.residual_x_with_secondary_N,mode="lines",name=f"{balance_pct}% balance incl. secondary"))
    fig.update_layout(template="plotly_dark",title="Line-of-stroke shaking-force comparison",height=380,
        paper_bgcolor="#111c2e",plot_bgcolor="#111c2e",xaxis_title="Crank angle (degrees)",yaxis_title="Force (N)")
    st.plotly_chart(fig,use_container_width=True)
    st.subheader("Full-cycle calculation data")
    st.dataframe(dyn.round(4),use_container_width=True,height=330)
    st.download_button("⬇ Download full-cycle results (CSV)",data=export_dataframe(dyn),
        file_name="ic_engine_dynamics_results.csv",mime="text/csv")

st.divider()
st.caption("Educational model: rigid links, constant crank speed, no friction or clearance, and simplified gas loading. Validate sign conventions and balancing assumptions against your course reference before academic submission.")
