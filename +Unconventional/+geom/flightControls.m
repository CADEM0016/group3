%% NOT DOING FOR NOW TOO LONG LOL

% function [massObj,GeomObj] = flightControls(obj)
% %FLIGHTCONTROLS Summary of this function goes here
% %   Detailed explanation goes here
% % Flight Controls
% 
% % ------------------------- Create Mass Objects --------------------------
% % VERY rough guesses for flight control weight distribution
% m_wing = 0.55 * mass_fc_kg;
% m_tail = 0.35 * mass_fc_kg;
% m_fuse = 0.10 * mass_fc_kg;
% 
% mass_fc_lb = 145.9 *c.N_f^0.554 *(1 + c.N_m / c.N_f)^(-1.0)*c.S_cs^0.20 *(c.I_yaw * 1e-6)^0.07;
% mass_fc_kg = W_fc_lb / SI.lb;
% 
% Masses(end+1) = cast.MassObj("Wing Flight Controls", m_wing, [obj.x_wing_ac; 0]);
% Masses(end+1) = cast.MassObj("Tail Flight Controls", m_tail, [obj.x_tail_ac; 0]);
% Masses(end+1) = cast.MassObj("Flight Control Systems", m_fuse, [obj.x_cg_est; 0]);
% 
% % --------------------------- Create Geometry ----------------------------
% 
% end