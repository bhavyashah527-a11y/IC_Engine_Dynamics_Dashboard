# IC Engine Dynamics & Crankshaft Balancing Simulator

## Project files
- `app.py`: Streamlit dashboard and Plotly visualizations.
- `engine_model.py`: independent calculation functions.
- `requirements.txt`: Python dependencies.

## Run locally
1. Install Python 3.10+.
2. Open a terminal in this folder.
3. (Recommended) Create a virtual environment:
   - Windows: `python -m venv .venv` then `.venv\\Scripts\\activate`
   - macOS/Linux: `python3 -m venv .venv` then `source .venv/bin/activate`
4. Install: `pip install -r requirements.txt`
5. Run: `streamlit run app.py`
6. Open the local URL Streamlit prints (normally http://localhost:8501).

## Model scope and assumptions
- Single-cylinder slider-crank with rigid links and constant crank speed.
- Exact slider-crank position derivatives for piston velocity and acceleration.
- Standard approximate primary and secondary reciprocating inertia-force harmonics.
- Gas pressure is a constant illustrative value, not a combustion-pressure trace.
- Counterweight estimate: `m_b*r_b = (m_rot + f*m_rec)*r`.
- A single rotating counterweight cannot eliminate all reciprocating shaking forces.
- No friction, bearing loads, crankshaft flexibility, torsional vibration, valve timing or multi-cylinder phasing.

## Academic validation
Check the balancing convention and force sign convention against your course notes. The model is intended as an educational starting point, not a validated engine design tool.
