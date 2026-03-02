function [] = empennage(obj)
%EMPENNAGE Summary of this function goes here
%   Detailed explanation goes here



L_ht = 0.42*obj.FuselageLength ; % horizontal tail moment arm
L_vt = 0.55*obj.FuselageLength; % vertical tail moment arm

% Tail Areas
S_Vt = (Span*WingAreaActual*obj.V_VT)/(L_vt) ; % Calculated vertical tail area (m^2)
S_Ht = (Cmac*WingAreaActual*obj.V_HT)/(L_ht) ; % Calculated horizontal tail area (m^2)

% Tail Spans
b_Ht = sqrt(obj.HT_AR * S_Ht); % Horizontal tail span (m)
b_Vt = sqrt(obj.VT_AR * S_Vt); % Vertical tail span (m)

% Root Chords
Croot_Ht = (2 * S_Ht) / (b_Ht * (1 + obj.HT_TR)); % Horizontal tail root chord (m)
Croot_Vt = (2 * S_Vt) / (b_Vt * (1 + obj.VT_TR)); % Vertical tail root chord (m)

% Tip Chords
Ctip_Ht = Croot_Ht * obj.HT_TR; % Horizontal tail tip chord (m)
Ctip_Vt = Croot_Vt * obj.VT_TR; % Vertical tail tip chord (m)

% Mean Aerodynamic Chords (MAC)
Cmac_Ht = (2 / 3) * Croot_Ht * (1 + obj.HT_TR + obj.HT_TR^2) / (1 + obj.HT_TR); % Horizontal tail MAC (m)
Cmac_Vt = (2 / 3) * Croot_Vt * (1 + obj.VT_TR + obj.VT_TR^2) / (1 + obj.VT_TR); % Vertical tail MAC (m)


end