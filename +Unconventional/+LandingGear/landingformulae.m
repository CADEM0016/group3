
CADEM0016 – MSc Group Design Project

Landing Gear Sizing:
Formulae & Calculations Reference

University of Bristol  |  Aerospace Engineering
Landing Gear Engineer  |  March 2026




1.  Aircraft Top-Level Inputs
The following top-level parameters are used throughout all three fidelity levels. Values are provided by cross-discipline collaboration.

Parameter	Symbol	Value	Source / Note
Max Take-Off Mass	MTOM	497,870 kg	MDO top-level hyperparameter
Max Landing Mass ratio	MLM / MTOM	0.90	Class I statistical ratio (§2.1)
CG height above ground	h_cg	3.15 m	Fuselage & Stability discipline
No. of MLG legs	n_legs	4	Initial selection (§3, widebody)
Fuselage length	l_fus	76.0 m	Fuselage Structures discipline
Engine fan diameter	D_fan	3.20 m	PPC (Performance) discipline
Steel yield strength	σ_y	1,000 MPa	300M high-strength steel
Steel Young's modulus	E	200 GPa	Standard structural steel
Steel density	ρ	7,850 kg/m³	Standard structural steel
Gravitational acceleration	g	9.81 m/s²	Constant

2.  Class I – Statistical / Empirical Formulae
Class I methods use historical correlations for rapid conceptual estimates (MDO Development sprint, initial sizing).

2.1  Maximum Landing Mass (MLM)
Historical widebody freighter ratio (B777F ≈ 0.86, A350 ≈ 0.89; 0.90 adopted as conservative upper bound):

MLM  =  MLM_ratio  ×  MTOM  =  0.90 × 497,870  =  448,083 kg

2.2  Total Landing Gear Mass
Raymer Table 15.2: transport aircraft landing gear fraction ≈ 3.3 – 5.7% of MTOM; 4% adopted:

M_LG  =  0.04 × MTOM  =  0.04 × 497,870  =  19,915 kg

The 90/10 empirical split (main/nose) reflects the 85–95% main-gear load-share requirement (Lecture Slide 7):

M_main  =  0.90 × M_LG  =  17,923 kg
M_nose  =  0.10 × M_LG  =  1,992 kg

2.3  Number of Main-Gear Wheels
Target ≈ 25,000 kg per tyre (A380 baseline: ~28,750 kg/tyre); must satisfy 2 wheels/axle civil certification rule (Lecture Slide 6):

n_wheels  =  ceil( 0.90 × MTOM  /  25,000 )  → rounded up to nearest even integer

n_wheels  =  ceil( 0.90 × 497,870 / 25,000 )  =  ceil(17.9)  =  18 wheels

3.  Class II – Semi-Empirical Formulae
Class II methods combine empirical correlations with simplified geometry and loads (MDO Development sprint, refined sizing).

3.1  Tyre Sizing  (Raymer Table 11.1)
Power-law empirical fit for Type VII transport tyres (Raymer, Aircraft Design: A Conceptual Approach, 6th ed.):

D_tyre [in]  =  0.89 × W_wheel [lbs] ^ 0.361
W_tyre [in]  =  0.35 × D_tyre [in]

Where the wheel load is:
W_wheel  =  ( 0.90 × MTOM × 2.2046 )  /  n_MLG_wheels

Tyre deflection under load (narrowbody and above, Lecture Slide 15):
x_t  =  6 in  =  0.152 m

3.2  Static Load Distribution
Empirical constraint: 85–95% of static load on main gear (Lecture Slide 7; Raymer p.340):

F_main  =  0.90 × MTOM × g  (total over all MLG legs)
F_nose  =  0.10 × MTOM × g
F_per_leg  =  F_main  /  n_legs

Verification by moment balance about the MLG (ensures correct longitudinal placement):

F_nose  =  MTOM × g × (x_mg − x_cg)  /  (x_mg − x_nlg)

3.3  Tail-Strike Angle  (Lecture Slide 7 & 8)
Geometric constraint; the angle between the ground line and the line from the main-gear tyres to the first fuselage contact point during rotation (target: 8–12°, hard limit: < 15°):

θ_ts  =  arctan( h_tail  /  dist_mg_to_tail )

3.4  Tip-Back Angle  (Lecture Slide 7 & 8)
The angle between vertical through MLG and the line to the worst-case aft CG. Must satisfy β > θ_ts and β < 25° (Lecture Slide 8):

β  =  arctan( (x_mg − x_cg_aft)  /  h_cg )

3.5  Turnover Angle  (Lecture Slide 9)
Indicator of aircraft tip-over resistance during a ground turn; must be < 63° (Lecture Slide 9):

α  =  arctan( T / (2 × B) )
Y  =  D × sin(α)
θ_to  =  arctan( E  /  Y )

Where: T = track width [m], B = wheelbase [m], D = longitudinal NLG-to-CG distance [m], E = CG height [m]

3.6  Landing Gear Length – Engine Clearance  (Lecture Slide 10)
Minimum gear length so the engine nacelle clears the ground (Lecture Slide 10):

h_gear_engine  =  h_fus_bottom  +  0.23 × D_fan

4.  Class II.5 – Physics-Based Formulae
Class II.5 methods use physics-based simplified models calibrated to empirical data (MDO Refinement sprint).

4.1  Shock Absorber Stroke Length  (Lecture Slide 15)
Energy balance at landing touchdown (assuming zero wing lift at touchdown, as per conservative certification assumption):

(1/2) × m × V_z²  =  λ × m × g × (0.75 × x_s  +  0.5 × x_t)

Rearranging for stroke length x_s:

x_s  =  [ V_z² / (2 λ g)  −  0.5 × x_t ]  /  0.75

Symbol	Value	Definition
V_z	3.048 m/s	Vertical descent rate at touchdown (certification: 10 ft/s)
m	MLM = 448,083 kg	Maximum Landing Mass (worst case)
λ	1.1 – 1.8	Landing reaction factor (large civil: 1.1–1.3; regional: 1.5–1.8)
x_t	0.152 m (6 in)	Tyre deflection (narrowbody and above, Lecture Slide 15)
0.75	–	Oleo-pneumatic shock absorber efficiency (absorption coefficient)
0.5	–	Tyre absorption efficiency coefficient

At the design point λ = 1.2:  x_s = 0.4193 m  (16.5 in)

4.2  Peak Landing Force per Leg
Peak force transmitted from each MLG leg into the airframe structure:

F_peak_leg  =  λ × (MLM × g × main_share)  /  n_legs

This is the critical structural load passed to the Wing Structures Engineer for the landing load case.

4.3  Slider (Piston) Diameter  (Lecture Slide 18)
The piston cross-sectional area is sized from the static nitrogen gas pressure when the aircraft is stationary at MTOW (Lecture Slide 18):

P_static  =  1,500 psi  =  10.34 MPa

F_static_leg  =  (MLM × g × 0.90)  /  n_legs
A_slider  =  F_static_leg  /  P_static
d_slider  =  2 × √( A_slider / π )

4.4  Main Fitting Length  (Lecture Slide 18)
The outer cylinder (main fitting) must accommodate the full stroke plus a 10% margin (Lecture Slide 18):

L_tube  =  1.10 × x_s

4.5  Main Fitting Wall Thickness – Euler Buckling
The outer cylinder is modelled as a hollow tube (fixed-free column, K = 2.0) sized to resist Euler buckling under peak landing load with a safety factor of 2.0:

P_cr  =  π² × E × I  /  (K × L)²

Required second moment of area:
I_req  =  SF × F_peak × (K × L)²  /  (π² × E)

For a hollow cylinder with outer diameter D_o (≈ 1.6 × d_slider):
D_i  =  ( D_o⁴  −  64 × I_req / π )^(1/4)
t_wall  =  (D_o  −  D_i)  /  2

Compressive stress check (controls if buckling does not):
σ_c  =  F_peak / A_tube  ≤  σ_y / SF

4.6  Physics-Based Leg Mass
Structural tube volume and mass:

V_tube  =  (π/4) × (D_o²  −  D_i²)  ×  L_tube
M_fitting  =  ρ_steel × V_tube

Total leg mass including piston and brackets (+50% system allowance):
M_leg  =  M_fitting × 1.50

M_LG_main  =  n_legs × M_leg

4.7  Raymer λ-Corrected Mass  (Schmidt p.865)
Cross-check using Raymer's transport-jet regression (as presented in Schmidt, The Design of Aircraft Landing Gear, p.865). Accounts explicitly for λ:

W_MLG [lbs]  =  0.0117 × λ × W_land^0.95 × L_gear^0.43 × N_main^0.525
W_NLG [lbs]  =  0.048  ×       W_land^0.67 × L_nlg^0.43  × N_nose^0.525

Symbol	Definition
W_land [lbs]	Maximum Landing Weight in pounds  =  MLM × 2.2046
L_gear [in]	MLG length in inches  =  (h_gear_engine + x_s) / 0.0254
L_nlg [in]	NLG length in inches  ≈  0.80 × L_gear
N_main	Number of main-gear wheels
N_nose	Number of nose-gear wheels
λ	Landing reaction factor (design choice; large civil ≈ 1.1–1.3)

5.  Economic Evaluation Formulae  (Specification §4)
These are used in collaboration with the Economics Engineer and are included here for completeness of the MDO framework.

5.1  Hull Value
V_hull  =  44,880 × MTOM^0.65   [USD]   (MTOM in kg)

5.2  Maintenance Costs
C_m,fixed  =  0.03 × V_hull   [USD/year]
C_m,var    =  5 × 10⁻⁶ × V_hull   [USD/FH]

5.3  Insurance
C_ins  =  0.005 × V_hull  =  224.4 × MTOM^0.65   [USD/year]

6.  Mass Estimate Comparison Table
Results at the design point (MTOM = 497,870 kg, λ = 1.2, n_legs = 4, n_wheels = 18 + 2 nose):

Method / Fidelity	Total LG Mass	Main Gear	% MTOM
Class I  (4% fraction)	19,915 kg	17,923 kg	4.00%
Class II.5  (tube physics)	~19,700 kg	~17,730 kg	~3.96%
Raymer λ-corrected  (Schmidt p.865)	~20,100 kg	~18,090 kg	~4.04%
B777F reference (~4% of 348.7 t)	~13,950 kg	~12,555 kg	4.00%

7.  References
[1] Raymer, D. P., Aircraft Design: A Conceptual Approach, 6th ed., AIAA, 2018.
[2] Schmidt, R. K., The Design of Aircraft Landing Gear, SAE International, 2021. (Sections 13.5.1, p.863–865)
[3] Currey, N. S., Aircraft Landing Gear Design: Principles and Practices, AIAA, 1988.
[4] Torenbeek, E., Advanced Aircraft Design, Wiley, 2013.
[5] Hoole, J., Landing Gear Design – Aerospace MSc GDP Lecture Slides, University of Bristol, 2026.
[6] Healy, F. & Poole, D., MSc Group Design Specification 2025 (CADEM0016), University of Bristol, 2026.
[7] Healy, F. & Poole, D., MSc Group Design Handbook (CADEM0016), University of Bristol, 2026.
