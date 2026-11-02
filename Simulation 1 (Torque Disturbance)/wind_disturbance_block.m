function [dFxyz, dMxyz, windDisturb6] = fcn(t, distMode, distFactor)
% Paper-style wind/gust disturbance generator for the CURRENT model wiring.
%
% IMPORTANT FOR YOUR CURRENT SIMULINK MODEL:
% This version uses SCALAR outputs, so you do NOT need Demux blocks and you
% do NOT need to change the plant force input wiring.
%
% Output-port usage for your current wiring:
%   dFxyz        -> connect to the U2 add block  = roll moment disturbance
%   dMxyz        -> connect to the U3 add block  = pitch moment disturbance
%   windDisturb6 -> connect to the U4 add block  = yaw moment disturbance
%
% Inputs:
%   t          : Clock signal, seconds
%   distMode   : 0=off, 1=mild, 2=paper/strong, 3=custom
%   distFactor : multiplier for all disturbance amplitudes
%
% Paper-style disturbance shape:
%   - step/gust impacts at 6 s, 10 s, and 15 s
%   - sinusoidal airflow at 0.5 Hz from 20 s to 30 s
%
% Note:
% The paper describes angular disturbances. In this 6DOF plant, the same
% time profile is converted to small moment disturbances in N.m.

mode = round(distMode);
if mode < 0
    mode = 0;
end

factor = abs(distFactor);

% ------------------------------------------------------------
% Paper-style timing and normalized amplitudes
% ------------------------------------------------------------
stepAmpPaper = 1.5;       % normalized step/gust amplitude from paper
sineAmpPaper = 1.0;       % normalized sine amplitude from paper
sineFreqHz   = 0.5;       % paper sine frequency, Hz
sineOmega    = 2*pi*sineFreqHz;

t1 = 6.0;
t2 = 10.0;
t3 = 15.0;
gustWidth = 0.80;         % seconds, gust pulse duration

sineStart = 20.0;
sineEnd   = 30.0;

% ------------------------------------------------------------
% Convert normalized paper disturbance to moment disturbance.
% Use distMode = 2 for the closest paper-like strong test.
% ------------------------------------------------------------
if mode == 0
    M_step_scale = 0.0;
    M_sine_scale = 0.0;
elseif mode == 1
    % Mild wind disturbance
    M_step_scale = 0.0012;    % N.m per normalized paper unit
    M_sine_scale = 0.0008;    % N.m per normalized paper unit
elseif mode == 2
    % Strong paper-like wind disturbance
    M_step_scale = 0.0030;    % N.m per normalized paper unit
    M_sine_scale = 0.0020;    % N.m per normalized paper unit
else
    % Custom/intermediate wind disturbance
    M_step_scale = 0.0020;    % N.m per normalized paper unit
    M_sine_scale = 0.0012;    % N.m per normalized paper unit
end

% ------------------------------------------------------------
% Step/gust pulses at 6, 10, and 15 seconds
% ------------------------------------------------------------
g1 = 0.0;
g2 = 0.0;
g3 = 0.0;

if t >= t1 && t < (t1 + gustWidth)
    g1 = stepAmpPaper;
end

if t >= t2 && t < (t2 + gustWidth)
    g2 = stepAmpPaper;
end

if t >= t3 && t < (t3 + gustWidth)
    g3 = stepAmpPaper;
end

% ------------------------------------------------------------
% 0.5 Hz sinusoidal airflow from 20 s to 30 s
% ------------------------------------------------------------
s = 0.0;
s90 = 0.0;
sSlow = 0.0;

if t >= sineStart && t <= sineEnd
    tau = t - sineStart;
    s = sineAmpPaper * sin(sineOmega*tau);
    s90 = sineAmpPaper * sin(sineOmega*tau + pi/2);
    sSlow = sineAmpPaper * sin(0.70*sineOmega*tau + pi/4);
end

% ------------------------------------------------------------
% Roll, pitch, yaw moment disturbances.
% These three scalar outputs are designed for the existing U2/U3/U4 add blocks.
% ------------------------------------------------------------
d_roll  = factor * (M_step_scale*( g1 - 0.40*g2) + M_sine_scale*s);
d_pitch = factor * (M_step_scale*(-0.50*g1 + g2) + M_sine_scale*s90);
d_yaw   = factor * (M_step_scale*( 0.50*g3) + 0.40*M_sine_scale*sSlow);

% ------------------------------------------------------------
% Keep the existing output names, but now each output is scalar.
% ------------------------------------------------------------
dFxyz        = d_roll;   % existing output port 1 -> U2 disturbance
dMxyz        = d_pitch;  % existing output port 2 -> U3 disturbance
windDisturb6 = d_yaw;    % existing output port 3 -> U4 disturbance

end
