%% Initialization_3 - Quadrotor trajectory tracking with paper-style wind disturbance
% Updated for current model wiring: wind disturbance outputs connected to U2, U3, U4.

%% Planner

Tfinal = 150;  % Simulation time (s)

% Run control flags
autoRunSimulink = true;   % true: run the Simulink model automatically from this script
animate3D = true;         % true: show moving 3-D reference/actual trajectory animation
maxAnimationFrames = 600; % keep animation smooth and not too slow for long simulations

% Steady-state error settings
% The default uses the final 10 percent of the simulation as the steady-state window.
% To force a specific window, set steadyStateStartTime manually, for example 20.
steadyStateWindowPercent = 10;
steadyStateStartTime = [];


%% Paper-style wind/gust disturbance settings
% These variables are sent to the Wind_Disturbance_Generator MATLAB Function
% block through the Constant blocks named/valued distMode and distFactor.
%
% Current updated model wiring:
%   Wind block output dFxyz        -> U2 summation block = roll disturbance
%   Wind block output dMxyz        -> U3 summation block = pitch disturbance
%   Wind block output windDisturb6 -> U4 summation block = yaw disturbance
%   The same three outputs should also go to the out.disturb Mux for logging.
%
% Mode selection:
%   distMode = 0  -> no wind disturbance
%   distMode = 1  -> mild paper-style disturbance
%   distMode = 2  -> strong paper-style disturbance, closest to paper test
%   distMode = 3  -> custom/intermediate disturbance
%
% Paper-style timing:
%   step/gust pulses at 6 s, 10 s, and 15 s
%   0.5 Hz sinusoidal airflow from 20 s to 30 s
%
% Use distFactor < 1 if the UAV becomes unstable under strong disturbance.
distMode = 3;        % <-- SELECT DISTURBANCE MODE HERE
distFactor = 19.0;    % disturbance amplitude multiplier

% slctr selects the reference signal generated inside the MATLAB Function block:
%   slctr = 1 -> circular trajectory
%   slctr = 2 -> upward helix trajectory
%   slctr = 3 -> figure-eight trajectory
%   otherwise -> fixed point X0
slctr = 3;
w = [1 1 0.5];  % motion rates: [1 1 0.5]
% X0 = (rand(1,3)*2 + 0.1).*(-1*ones(1,3)).^(randi([1 2],1,3)); % Reference position in slctr = 2 mode
X0 = [10 10 5]; % target point: [5 5 10]

% Controller tuning profile
% -------------------------------------------------------------------------
% The previous high-gain tuning attempt increased the 3-D RMSE slightly.
% Therefore the default is restored to the original/baseline gain set.  Use
% this profile first, then test the mild profile only if you want a small
% attitude/yaw correction.
%
%   'baseline'   -> original stable gains from the earlier model
%   'mild'       -> small attitude/yaw improvement, position gains mostly kept
%   'aggressive' -> high-gain attempt; kept only for comparison
%
tuningProfile = 'baseline';


%% Parameters of Drone
g = 9.8; 
mass = 0.65;  % Mass of drone (kg)

% Moment of inertia
Ix = 7.5e-3;  
Iy = 7.5e-3;
Iz = 1.3e-2;

Jr = 6e-5; % rotor inertia
l = 0.23; % arm length

b = 3.13e-5; % thrust coefficient
d = 7.5e-7; % drag coefficient

initpos = [0 0 0]; % Drone's initial position
initvb = [0 0 0]; % Drone's initial velocity in body frame
initeuler = [0 0 0]; % Drone's initial euler angels
initangRates = [0 0 0]; % Drone's initial angular volocity in body frame
inertia_mat = [Ix 0 0
               0 Iy 0
               0 0 Iz];

%% NTSMC parameters

% Controller gain selection
% -------------------------------------------------------------------------
% IMPORTANT: the last aggressive update reduced a few channels but increased
% Y, Z, pitch and total 3-D RMSE.  A gain increase is not always an
% improvement because it can add overshoot and excite the attitude loop.
% This switch keeps the stable baseline as the default and lets you compare
% small/mild and aggressive alternatives without editing many lines.

switch lower(tuningProfile)
    case 'baseline'
        % Original stable gain set.  This is the recommended default because
        % your latest result showed the aggressive values were worse overall.
        kp_z = -15;    kp_y = -2.6224; kp_x = -2.9805;
        kd_z = -20;    kd_y = -2.0093; kd_x = -2.6701;

        kk2 = 15;
        kk3 = 15;
        beta = 2;

        k_psi1 = 8;
        k_psi2 = 8;

        fuzzyGainValues = [5 10 15 20 26 28 30 35 40 50 55 60];

    case 'mild'
        % Small correction only.  Position-loop gains are kept close to the
        % baseline, while attitude/yaw gains are increased slightly.  Use this
        % only after the baseline result is confirmed.
        kp_z = -15;    kp_y = -2.7;    kp_x = -3.0;
        kd_z = -20;    kd_y = -2.3;    kd_x = -3.0;

        kk2 = 16;
        kk3 = 16;
        beta = 2.2;

        k_psi1 = 9;
        k_psi2 = 9;

        fuzzyGainValues = [6 11 16 21 26 28 30 35 40 50 55 60];

    case 'aggressive'
        % High-gain comparison profile.  This profile produced higher total
        % 3-D RMSE in your last test, so it is not recommended as default.
        kp_x = -3;     kp_y = -3;      kp_z = -12;
        kd_x = -4;     kd_y = -4;      kd_z = -18;

        kk2 = 18;
        kk3 = 18;
        beta = 2.5;

        k_psi1 = 10;
        k_psi2 = 10;

        fuzzyGainValues = [8 12 18 22 26 28 30 35 40 50 55 60];

    otherwise
        error('Unknown tuningProfile "%s". Use baseline, mild, or aggressive.', tuningProfile);
end

% Assign attitude gains used by the Simulink model.
k_phi1 = kk2;
k_phi2 = kk3;

k_theta1 = kk2;
k_theta2 = kk3;

a1 = (Iy-Iz)/Ix;
a2 = Jr/Ix;
a3 = (Iz-Ix)/Iy;
a4 = Jr/Iy;
a5 = (Ix-Iy)/Iz;

Omega2U = [b b b b
           0 -b 0 b
          -b 0 b 0
          -d d -d d];
U2Omega = inv(Omega2U);    %% convert controller outputs → actual motor speedsconvert controller outputs → actual motor speeds
%%  NTSMC parameters
gama = 1.5; % 1 < gama < 2
ro = 0.3; % 0 < ro < 1
beta_phi = beta; beta_theta = beta; beta_psi = beta;

%% Disturbance observer tuning parameters
% These are provided for models that include finite-time disturbance
% observer blocks.  They do not change the simulation unless the Simulink
% model uses the variables kd1, kd2, and gamma_d.  The values follow the
% observer tuning style used in the reference paper.
kd1 = diag([4 4 4 4 4 4]);
kd2 = diag([200 200 200 200 200 200]);
gamma_d = 5/7;

%% NTSMC Adaptive Gain Scheduling with Fuzzy Inference System
% Define FIS
    fis_theta = sugfis();
    fis_theta=addInput(fis_theta,[-10 10],'Name','s_theta');
    fis_theta=addInput(fis_theta,[-3 3],'Name','s_theta_dot');
    fis_theta=addInput(fis_theta,[0 pi],'Name','theta_abs');
    fis_theta=addInput(fis_theta,[0 pi],'Name','phi_abs');
    fis_theta = addOutput(fis_theta,[0 80],'Name',"k_theta");
    
    fis_phi = sugfis();
    fis_phi=addInput(fis_phi,[-10 10],'Name','s_phi');
    fis_phi=addInput(fis_phi,[-3 3],'Name','s_phi_dot');
    fis_phi=addInput(fis_phi,[0 pi],'Name','phi_abs');
    fis_phi=addInput(fis_phi,[0 pi],'Name','theta_abs');
    fis_phi = addOutput(fis_phi,[0 50],'Name',"k_phi");

% Add Membership functions
    % FIS theta
    fis_theta = addMF(fis_theta,"s_theta","trapmf",[-20 -20 -0.2 -0.2]);
    fis_theta = addMF(fis_theta,"s_theta","trimf",[-0.2 -0.1 0]);
    fis_theta = addMF(fis_theta,"s_theta","trimf",[-0.1 0 0.1]);
    fis_theta = addMF(fis_theta,"s_theta","trimf",[0 0.1 0.2]);
    fis_theta = addMF(fis_theta,"s_theta","trapmf",[0.1 0.2 20 20]);

    fis_theta = addMF(fis_theta,"s_theta_dot","trapmf",[-5 -5 -0.1 0]);
    fis_theta = addMF(fis_theta,"s_theta_dot","trimf",[-0.1 0 0.1]);
    fis_theta = addMF(fis_theta,"s_theta_dot","trapmf",[0 0.1 5 5]);

    fis_theta = addMF(fis_theta,"theta_abs","trapmf",[0 0 0.05 0.2]);
    fis_theta = addMF(fis_theta,"theta_abs","trimf",[0.05 0.2 0.5]);
    fis_theta = addMF(fis_theta,"theta_abs","trapmf",[0.2 0.5 pi pi]);

    fis_theta = addMF(fis_theta,"phi_abs","trapmf",[0 0 0.05 0.2]);
    fis_theta = addMF(fis_theta,"phi_abs","trimf",[0.05 0.2 0.5]);
    fis_theta = addMF(fis_theta,"phi_abs","trapmf",[0.2 0.5 pi pi]);

    % FIS phi
    fis_phi = addMF(fis_phi,"s_phi","trapmf",[-20 -20 -0.2 -0.1]);
    fis_phi = addMF(fis_phi,"s_phi","trimf",[-0.2 -0.1 0]);
    fis_phi = addMF(fis_phi,"s_phi","trimf",[-0.1 0 0.1]);
    fis_phi = addMF(fis_phi,"s_phi","trimf",[0 0.1 0.2]);
    fis_phi = addMF(fis_phi,"s_phi","trapmf",[0.1 0.2 20 20]);

    fis_phi = addMF(fis_phi,"s_phi_dot","trapmf",[-5 -5 -0.1 0]);
    fis_phi = addMF(fis_phi,"s_phi_dot","trimf",[-0.1 0 0.1]);
    fis_phi = addMF(fis_phi,"s_phi_dot","trapmf",[0 0.1 5 5]);

    fis_phi = addMF(fis_phi,"phi_abs","trapmf",[0 0 0.05 0.2]);
    fis_phi = addMF(fis_phi,"phi_abs","trimf",[0.05 0.2 0.5]);
    fis_phi = addMF(fis_phi,"phi_abs","trapmf",[0.2 0.5 pi pi]);

    fis_phi = addMF(fis_phi,"theta_abs","trapmf",[0 0 0.05 0.2]);
    fis_phi = addMF(fis_phi,"theta_abs","trimf",[0.05 0.2 0.5]);
    fis_phi = addMF(fis_phi,"theta_abs","trapmf",[0.2 0.5 pi pi]);

% MF's for output(u)
    % Output adaptive-gain values are selected by tuningProfile above.  The
    % rule matrix below uses output membership indices 1..12, so both FIS
    % systems must receive exactly 12 output MFs in the same order.
    for mfIdx = 1:numel(fuzzyGainValues)
        fis_theta = addMF(fis_theta,"k_theta","constant",fuzzyGainValues(mfIdx));
        fis_phi   = addMF(fis_phi,  "k_phi",  "constant",fuzzyGainValues(mfIdx));
    end

% Rules of FIS
    rules = [1 1 1 1 10 1 1
             1 1 2 1 10 1 1
             1 1 2 2 11 1 1
             1 1 1 2 11 1 1
             1 1 3 1 12 1 1
             1 1 3 2 12 1 1
             1 1 3 3 12 1 1
             1 1 1 3 12 1 1
             1 1 2 3 12 1 1
             1 2 1 1 8 1 1
             1 2 2 1 8 1 1
             1 2 2 2 9 1 1
             1 2 1 2 9 1 1
             1 2 3 1 10 1 1
             1 2 3 2 10 1 1
             1 2 3 3 10 1 1
             1 2 1 3 10 1 1
             1 2 2 3 10 1 1
             1 3 1 1 9 1 1
             1 3 2 1 9 1 1
             1 3 2 2 9 1 1
             1 3 1 2 9 1 1
             1 3 3 1 10 1 1
             1 3 3 2 10 1 1
             1 3 3 3 10 1 1
             1 3 1 3 10 1 1
             1 3 2 3 10 1 1
             2 1 1 1 8 1 1
             2 1 2 1 9 1 1
             2 1 2 2 9 1 1
             2 1 1 2 9 1 1
             2 1 3 1 10 1 1
             2 1 3 2 10 1 1
             2 1 3 3 10 1 1
             2 1 1 3 10 1 1
             2 1 2 3 10 1 1
             2 2 1 1 5 1 1
             2 2 2 1 6 1 1
             2 2 2 2 6 1 1
             2 2 1 2 6 1 1
             2 2 3 1 7 1 1
             2 2 3 2 7 1 1
             2 2 3 3 7 1 1
             2 2 1 3 7 1 1
             2 2 2 3 7 1 1
             2 3 1 1 4 1 1
             2 3 2 1 5 1 1
             2 3 2 2 5 1 1
             2 3 1 2 5 1 1
             2 3 3 1 6 1 1
             2 3 3 2 6 1 1
             2 3 3 3 6 1 1
             2 3 1 3 6 1 1
             2 3 2 3 6 1 1
             3 1 1 1 5 1 1
             3 1 2 1 6 1 1
             3 1 2 2 6 1 1
             3 1 1 2 6 1 1
             3 1 3 1 7 1 1
             3 1 3 2 7 1 1
             3 1 3 3 7 1 1
             3 1 1 3 7 1 1
             3 1 2 3 7 1 1
             3 2 1 1 1 1 1
             3 2 2 1 2 1 1
             3 2 2 2 2 1 1
             3 2 1 2 2 1 1
             3 2 3 1 3 1 1
             3 2 3 2 3 1 1
             3 2 3 3 3 1 1
             3 2 1 3 3 1 1
             3 2 2 3 3 1 1
             3 3 1 1 5 1 1
             3 3 2 1 6 1 1
             3 3 2 2 6 1 1
             3 3 1 2 6 1 1
             3 3 3 1 7 1 1
             3 3 3 2 7 1 1
             3 3 3 3 7 1 1
             3 3 1 3 7 1 1
             3 3 2 3 7 1 1
             4 1 1 1 8 1 1
             4 3 2 1 9 1 1
             4 3 2 2 9 1 1
             4 3 1 2 9 1 1
             4 3 3 1 10 1 1
             4 3 3 2 10 1 1
             4 3 3 3 10 1 1
             4 3 1 3 10 1 1
             4 3 2 3 10 1 1
             4 2 1 1 5 1 1
             4 2 2 1 6 1 1
             4 2 2 2 6 1 1
             4 2 1 2 6 1 1
             4 2 3 1 7 1 1
             4 2 3 2 7 1 1
             4 2 3 3 7 1 1
             4 2 1 3 7 1 1
             4 2 2 3 7 1 1
             4 1 1 1 4 1 1
             4 1 2 1 5 1 1
             4 1 2 2 5 1 1
             4 1 1 2 5 1 1
             4 1 3 1 6 1 1
             4 1 3 2 6 1 1
             4 1 3 3 6 1 1
             4 1 1 3 6 1 1
             4 1 2 3 6 1 1
             5 3 1 1 10 1 1
             5 3 2 1 10 1 1
             5 3 2 2 11 1 1
             5 3 1 2 11 1 1
             5 3 3 1 12 1 1
             5 3 3 2 12 1 1
             5 3 3 3 12 1 1
             5 3 1 3 12 1 1
             5 3 2 3 12 1 1
             5 2 1 1 10 1 1
             5 2 2 1 10 1 1
             5 2 2 2 10 1 1
             5 2 1 2 10 1 1
             5 2 3 1 10 1 1
             5 2 3 2 10 1 1
             5 2 3 3 10 1 1
             5 2 1 3 10 1 1
             5 2 2 3 10 1 1
             5 1 1 1 8 1 1
             5 1 2 1 8 1 1
             5 1 2 2 8 1 1
             5 1 1 2 8 1 1
             5 1 3 1 8 1 1
             5 1 3 2 8 1 1
             5 1 3 3 8 1 1
             5 1 1 3 8 1 1
             5 1 2 3 8 1 1
             ];

fis_theta = addRule(fis_theta,rules);
fis_phi = addRule(fis_phi,rules);

%% Run Simulink model automatically
% This section runs the Simulink model after all parameters/FIS objects are created.
% The To Workspace outputs are returned in the SimulationOutput object named "out".

modelName = 'Adaptive_fuzzy_NTSMC';
scriptFolder = fileparts(mfilename('fullpath'));
if ~isempty(scriptFolder) && isfolder(scriptFolder)
    addpath(scriptFolder);
end

if autoRunSimulink
    if ~bdIsLoaded(modelName)
        modelFile = fullfile(scriptFolder, [modelName '.slx']);
        if ~isempty(scriptFolder) && exist(modelFile,'file')
            load_system(modelFile);
        else
            load_system(modelName);
        end
    end

    set_param(modelName, 'StopTime', num2str(Tfinal));
    fprintf('\nRunning Simulink model: %s\n', modelName);
    out = sim(modelName, ...
              'StopTime', num2str(Tfinal), ...
              'ReturnWorkspaceOutputs', 'on');
    fprintf('Simulation finished. Generating trajectory, error, and RMSE figures...\n');
else
    if ~exist('out','var')
        error('autoRunSimulink is false, but no simulation output variable named "out" exists.');
    end
end

%% Extract simulation outputs safely
% Required signals from the model: ref, pos, phi, theta, psi.
% ref = [x_ref y_ref z_ref], pos = [x y z]

time = localGetTime(out);
time = time(:);

refData = localGetSignalData(out,'ref',numel(time));
posData = localGetSignalData(out,'pos',numel(time));
phiData = localGetSignalData(out,'phi',numel(time));
thetaData = localGetSignalData(out,'theta',numel(time));
psiData = localGetSignalData(out,'psi',numel(time));

% Use common length to avoid dimension mismatches.
n = min([numel(time), size(refData,1), size(posData,1), ...
         size(phiData,1), size(thetaData,1), size(psiData,1)]);
time = time(1:n);
refData = refData(1:n,:);
posData = posData(1:n,:);
phiData = phiData(1:n,:);
thetaData = thetaData(1:n,:);
psiData = psiData(1:n,:);

x_ref = refData(:,1);  y_ref = refData(:,2);  z_ref = refData(:,3);
x     = posData(:,1);  y     = posData(:,2);  z     = posData(:,3);

phi = phiData(:,1);
if size(phiData,2) >= 2
    phi_ref = phiData(:,2);
else
    phi_ref = zeros(n,1);
end

theta = thetaData(:,1);
if size(thetaData,2) >= 2
    theta_ref = thetaData(:,2);
else
    theta_ref = zeros(n,1);
end

psi = psiData(:,1);
if size(psiData,2) >= 2
    psi_ref = psiData(:,2);
else
    psi_ref = zeros(n,1); % yaw reference block is set to 0 in the model
end

%% Tracking error calculation
% Error convention: error = reference - actual.

x_error = x_ref - x;
y_error = y_ref - y;
z_error = z_ref - z;

roll_error  = localAngleError(phi_ref, phi);
pitch_error = localAngleError(theta_ref, theta);
yaw_error   = localAngleError(psi_ref, psi);

error_3d = sqrt(x_error.^2 + y_error.^2 + z_error.^2);

%% Steady-state error calculation
% Steady-state error convention: mean absolute error over the selected
% steady-state time window.  The maximum absolute error over the same
% window is also calculated, because it is useful for reporting the
% remaining ripple/oscillation after convergence.

if isempty(steadyStateStartTime)
    steadyStateStartTime = time(1) + (1 - steadyStateWindowPercent/100) * (time(end) - time(1));
end

idx_ss = time >= steadyStateStartTime;
if ~any(idx_ss)
    warning('No samples found in the requested steady-state window. Using the full simulation instead.');
    idx_ss = true(size(time));
    steadyStateStartTime = time(1);
end

steadyStateWindow = [time(find(idx_ss,1,'first')), time(find(idx_ss,1,'last'))];

ss_x_mean = mean(abs(x_error(idx_ss)));
ss_y_mean = mean(abs(y_error(idx_ss)));
ss_z_mean = mean(abs(z_error(idx_ss)));
ss_roll_mean  = mean(abs(roll_error(idx_ss)));
ss_pitch_mean = mean(abs(pitch_error(idx_ss)));
ss_yaw_mean   = mean(abs(yaw_error(idx_ss)));
ss_3d_mean = mean(error_3d(idx_ss));

ss_x_max = max(abs(x_error(idx_ss)));
ss_y_max = max(abs(y_error(idx_ss)));
ss_z_max = max(abs(z_error(idx_ss)));
ss_roll_max  = max(abs(roll_error(idx_ss)));
ss_pitch_max = max(abs(pitch_error(idx_ss)));
ss_yaw_max   = max(abs(yaw_error(idx_ss)));
ss_3d_max = max(error_3d(idx_ss));

%% RMSE calculation
rmse_x = localRMSE(x_error);
rmse_y = localRMSE(y_error);
rmse_z = localRMSE(z_error);

rmse_roll  = localRMSE(roll_error);
rmse_pitch = localRMSE(pitch_error);
rmse_yaw   = localRMSE(yaw_error);

% 3-D position RMSE: Euclidean RMSE over x/y/z.
rmse_3d = localRMSE(error_3d);

tracking_rmse = struct();
tracking_rmse.x = rmse_x;
tracking_rmse.y = rmse_y;
tracking_rmse.z = rmse_z;
tracking_rmse.roll = rmse_roll;
tracking_rmse.pitch = rmse_pitch;
tracking_rmse.yaw = rmse_yaw;
tracking_rmse.position_3d = rmse_3d;

%% ITAE calculation
% Raw ITAE is integral(t*abs(error(t)) dt), with units error-unit*s^2.
% For easier comparison with RMSE/position error, the table and bar chart use
% a normalized ITAE-equivalent error:
%
%   ITAE_equivalent = integral(t*abs(error(t)) dt) / integral(t dt)
%
% This keeps the ITAE time-weighting but converts the value back to the
% original error unit: m for x/y/z/3D and rad for roll/pitch/yaw.
itae_x_raw = localITAE(time, x_error);
itae_y_raw = localITAE(time, y_error);
itae_z_raw = localITAE(time, z_error);

itae_roll_raw  = localITAE(time, roll_error);
itae_pitch_raw = localITAE(time, pitch_error);
itae_yaw_raw   = localITAE(time, yaw_error);
itae_3d_raw    = localITAE(time, error_3d);

itae_x = localITAEEquivalentError(time, x_error);
itae_y = localITAEEquivalentError(time, y_error);
itae_z = localITAEEquivalentError(time, z_error);

itae_roll  = localITAEEquivalentError(time, roll_error);
itae_pitch = localITAEEquivalentError(time, pitch_error);
itae_yaw   = localITAEEquivalentError(time, yaw_error);
itae_3d    = localITAEEquivalentError(time, error_3d);

tracking_itae = struct();
tracking_itae.equivalent_error.x = itae_x;
tracking_itae.equivalent_error.y = itae_y;
tracking_itae.equivalent_error.z = itae_z;
tracking_itae.equivalent_error.roll = itae_roll;
tracking_itae.equivalent_error.pitch = itae_pitch;
tracking_itae.equivalent_error.yaw = itae_yaw;
tracking_itae.equivalent_error.position_3d = itae_3d;
tracking_itae.raw.x = itae_x_raw;
tracking_itae.raw.y = itae_y_raw;
tracking_itae.raw.z = itae_z_raw;
tracking_itae.raw.roll = itae_roll_raw;
tracking_itae.raw.pitch = itae_pitch_raw;
tracking_itae.raw.yaw = itae_yaw_raw;
tracking_itae.raw.position_3d = itae_3d_raw;

Channel = {'X position'; 'Y position'; 'Z position'; ...
           'Roll angle'; 'Pitch angle'; 'Yaw angle'; '3D position'};
RMSE = [rmse_x; rmse_y; rmse_z; rmse_roll; rmse_pitch; rmse_yaw; rmse_3d];
SteadyStateMeanAbsError = [ss_x_mean; ss_y_mean; ss_z_mean; ...
                           ss_roll_mean; ss_pitch_mean; ss_yaw_mean; ss_3d_mean];
SteadyStateMaxAbsError = [ss_x_max; ss_y_max; ss_z_max; ...
                          ss_roll_max; ss_pitch_max; ss_yaw_max; ss_3d_max];
Unit = {'m'; 'm'; 'm'; 'rad'; 'rad'; 'rad'; 'm'};
RMSE_Table = table(Channel, RMSE, SteadyStateMeanAbsError, SteadyStateMaxAbsError, Unit);

ITAE_EquivalentError = [itae_x; itae_y; itae_z; itae_roll; itae_pitch; itae_yaw; itae_3d];
Raw_ITAE = [itae_x_raw; itae_y_raw; itae_z_raw; ...
            itae_roll_raw; itae_pitch_raw; itae_yaw_raw; itae_3d_raw];
ITAE_Unit = {'m'; 'm'; 'm'; 'rad'; 'rad'; 'rad'; 'm'};
Raw_ITAE_Unit = {'m*s^2'; 'm*s^2'; 'm*s^2'; ...
                 'rad*s^2'; 'rad*s^2'; 'rad*s^2'; 'm*s^2'};
ITAE_Table = table(Channel, ITAE_EquivalentError, ITAE_Unit, Raw_ITAE, Raw_ITAE_Unit);

PositionChannel = {'X position'; 'Y position'; 'Z position'; '3D position'};
MeanAbsSteadyStateError = [ss_x_mean; ss_y_mean; ss_z_mean; ss_3d_mean];
MaxAbsSteadyStateError = [ss_x_max; ss_y_max; ss_z_max; ss_3d_max];
UnitPosition = {'m'; 'm'; 'm'; 'm'};
SteadyStateError_Table = table(PositionChannel, MeanAbsSteadyStateError, ...
                               MaxAbsSteadyStateError, UnitPosition);

steady_state_error = struct();
steady_state_error.window_start_time = steadyStateWindow(1);
steady_state_error.window_end_time = steadyStateWindow(2);
steady_state_error.mean_abs.x = ss_x_mean;
steady_state_error.mean_abs.y = ss_y_mean;
steady_state_error.mean_abs.z = ss_z_mean;
steady_state_error.mean_abs.roll = ss_roll_mean;
steady_state_error.mean_abs.pitch = ss_pitch_mean;
steady_state_error.mean_abs.yaw = ss_yaw_mean;
steady_state_error.mean_abs.position_3d = ss_3d_mean;
steady_state_error.max_abs.x = ss_x_max;
steady_state_error.max_abs.y = ss_y_max;
steady_state_error.max_abs.z = ss_z_max;
steady_state_error.max_abs.roll = ss_roll_max;
steady_state_error.max_abs.pitch = ss_pitch_max;
steady_state_error.max_abs.yaw = ss_yaw_max;
steady_state_error.max_abs.position_3d = ss_3d_max;

time_s = time;
Error_Table = table(time_s, x_error, y_error, z_error, ...
                    roll_error, pitch_error, yaw_error, error_3d);

assignin('base','tracking_rmse',tracking_rmse);
assignin('base','tracking_itae',tracking_itae);
assignin('base','steady_state_error',steady_state_error);
assignin('base','RMSE_Table',RMSE_Table);
assignin('base','ITAE_Table',ITAE_Table);
assignin('base','SteadyStateError_Table',SteadyStateError_Table);
assignin('base','Error_Table',Error_Table);

fprintf('\n================ RMSE AND STEADY-STATE SUMMARY ================\n');
fprintf('Steady-state window: %.4f s to %.4f s\n', steadyStateWindow(1), steadyStateWindow(2));
disp(RMSE_Table);
fprintf('\n================ ITAE SUMMARY ================\n');
disp(ITAE_Table);
fprintf('\nPosition-channel steady-state error table:\n');
disp(SteadyStateError_Table);
fprintf('Variables saved in workspace: Error_Table, RMSE_Table, ITAE_Table, SteadyStateError_Table, tracking_rmse, tracking_itae, steady_state_error\n');
fprintf('================================================================\n');

%% Figure 1: Position tracking in x, y, z
figure('Name','Position Tracking: X/Y/Z','Color','w');
subplot(3,1,1)
plot(time,x_ref,'r--','LineWidth',1.2); hold on
plot(time,x,'b','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('X (m)'); title('X Position Tracking');
legend('X reference','X actual','Location','best')

subplot(3,1,2)
plot(time,y_ref,'r--','LineWidth',1.2); hold on
plot(time,y,'b','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('Y (m)'); title('Y Position Tracking');
legend('Y reference','Y actual','Location','best')

subplot(3,1,3)
plot(time,z_ref,'r--','LineWidth',1.2); hold on
plot(time,z,'b','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('Z (m)'); title('Z Position Tracking');
legend('Z reference','Z actual','Location','best')

%% Figure 2: Position tracking errors
figure('Name','Position Tracking Errors','Color','w');
subplot(3,1,1)
plot(time,x_error,'LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('e_x (m)'); title('X Tracking Error: x_{ref} - x')

subplot(3,1,2)
plot(time,y_error,'LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('e_y (m)'); title('Y Tracking Error: y_{ref} - y')

subplot(3,1,3)
plot(time,z_error,'LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('e_z (m)'); title('Z Tracking Error: z_{ref} - z')

%% Figure 3: Attitude tracking in roll, pitch, yaw
figure('Name','Attitude Tracking: Roll/Pitch/Yaw','Color','w');
subplot(3,1,1)
plot(time,phi_ref,'r--','LineWidth',1.2); hold on
plot(time,phi,'b','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('\phi (rad)'); title('Roll Tracking');
legend('Roll reference','Roll actual','Location','best')

subplot(3,1,2)
plot(time,theta_ref,'r--','LineWidth',1.2); hold on
plot(time,theta,'b','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('\theta (rad)'); title('Pitch Tracking');
legend('Pitch reference','Pitch actual','Location','best')

subplot(3,1,3)
plot(time,psi_ref,'r--','LineWidth',1.2); hold on
plot(time,psi,'b','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('\psi (rad)'); title('Yaw Tracking');
legend('Yaw reference','Yaw actual','Location','best')

%% Figure 4: Attitude tracking errors
figure('Name','Attitude Tracking Errors','Color','w');
subplot(3,1,1)
plot(time,roll_error,'LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('e_\phi (rad)'); title('Roll Error: \phi_{ref} - \phi')

subplot(3,1,2)
plot(time,pitch_error,'LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('e_\theta (rad)'); title('Pitch Error: \theta_{ref} - \theta')

subplot(3,1,3)
plot(time,yaw_error,'LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('e_\psi (rad)'); title('Yaw Error: \psi_{ref} - \psi')

%% Figure 5: 3-D reference tracking plot
figure('Name','3D Reference Tracking','Color','w');
plot3(x_ref,y_ref,z_ref,'r--','LineWidth',1.5); hold on
plot3(x,y,z,'b','LineWidth',1.5);
plot3(x_ref(1),y_ref(1),z_ref(1),'ko','MarkerSize',7,'MarkerFaceColor','k');
plot3(x(end),y(end),z(end),'mo','MarkerSize',7,'MarkerFaceColor','m');
grid on; axis equal; view(3)
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('3D Trajectory Tracking, 3D RMSE = %.6g m', rmse_3d));
legend('Reference trajectory','Quadcopter trajectory','Start reference','Final actual','Location','best')

%% Figure 6: Top-view trajectory plot (X-Y plane)
figure('Name','Top View Trajectory','Color','w');
plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5); hold on
plot(x, y, 'b', 'LineWidth', 1.5);
plot(x_ref(1), y_ref(1), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
plot(x(end), y(end), 'mo', 'MarkerSize', 7, 'MarkerFaceColor', 'm');
grid on; axis equal
xlabel('X (m)');
ylabel('Y (m)');
title(sprintf('Top View Trajectory Tracking, 3D RMSE = %.6g m', rmse_3d));
legend('Reference trajectory','Quadcopter trajectory','Start reference','Final actual','Location','best')

%% Figure 7: 3-D trajectory animation / simulation visualization
if animate3D
    figure('Name','3D Trajectory Animation','Color','w');
    hold on; grid on; axis equal; view(3)
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Trajectory Animation')

    refLine = animatedline('LineStyle','--','LineWidth',1.3,'Color','r');
    actualLine = animatedline('LineWidth',1.5,'Color','b');
    refMarker = plot3(nan,nan,nan,'ro','MarkerSize',7,'MarkerFaceColor','r');
    actualMarker = plot3(nan,nan,nan,'bo','MarkerSize',7,'MarkerFaceColor','b');
    legend('Reference trajectory','Quadcopter trajectory','Reference point','Actual point','Location','best')

    xAll = [x_ref; x]; yAll = [y_ref; y]; zAll = [z_ref; z];
    spanX = max(xAll) - min(xAll);
    spanY = max(yAll) - min(yAll);
    spanZ = max(zAll) - min(zAll);
    margin = 0.1*max([spanX, spanY, spanZ, 1]);
    xlim([min(xAll)-margin, max(xAll)+margin]);
    ylim([min(yAll)-margin, max(yAll)+margin]);
    zlim([min(zAll)-margin, max(zAll)+margin]);

    frameStep = max(1, floor(n/maxAnimationFrames));
    for k = 1:frameStep:n
        addpoints(refLine, x_ref(k), y_ref(k), z_ref(k));
        addpoints(actualLine, x(k), y(k), z(k));
        set(refMarker, 'XData', x_ref(k), 'YData', y_ref(k), 'ZData', z_ref(k));
        set(actualMarker, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
        title(sprintf('3D Trajectory Animation   t = %.2f s   e_{3D} = %.4g m', time(k), error_3d(k)));
        drawnow limitrate
    end
end

%% Figure 8: RMSE bar charts
figure('Name','RMSE Summary','Color','w');
subplot(2,1,1)
bar([rmse_x rmse_y rmse_z rmse_3d]); grid on
set(gca,'XTickLabel',{'X','Y','Z','3D'});
ylabel('RMSE (m)'); title('Position RMSE')

subplot(2,1,2)
bar([rmse_roll rmse_pitch rmse_yaw]); grid on
set(gca,'XTickLabel',{'Roll','Pitch','Yaw'});
ylabel('RMSE (rad)'); title('Attitude RMSE')

%% Figure 9: ITAE summary bar charts
figure('Name','ITAE Summary','Color','w');
subplot(2,1,1)
bar([itae_x itae_y itae_z itae_3d]); grid on
set(gca,'XTickLabel',{'X','Y','Z','3D'});
ylabel('ITAE-equivalent error (m)'); title('Position ITAE-equivalent Error')

subplot(2,1,2)
bar([itae_roll itae_pitch itae_yaw]); grid on
set(gca,'XTickLabel',{'Roll','Pitch','Yaw'});
ylabel('ITAE-equivalent error (rad)'); title('Attitude ITAE-equivalent Error')

%% Figure 10: Steady-state error summary for x/y/z channels
figure('Name','Steady-State Position Error Summary','Color','w');
subplot(2,1,1)
bar([ss_x_mean ss_y_mean ss_z_mean ss_3d_mean]); grid on
set(gca,'XTickLabel',{'X','Y','Z','3D'});
ylabel('Mean abs. steady-state error (m)');
title(sprintf('Steady-State Error, Window %.2f s to %.2f s', steadyStateWindow(1), steadyStateWindow(2)))

subplot(2,1,2)
bar([ss_x_max ss_y_max ss_z_max ss_3d_max]); grid on
set(gca,'XTickLabel',{'X','Y','Z','3D'});
ylabel('Max abs. steady-state error (m)');
title('Maximum Absolute Error Inside Steady-State Window')

%% Figure 11: Position errors with steady-state window highlighted
figure('Name','Position Errors with Steady-State Window','Color','w');
subplot(3,1,1)
plot(time,x_error,'LineWidth',1.2); hold on; grid on
plot(time(idx_ss),x_error(idx_ss),'r','LineWidth',1.4);
xlabel('Time (s)'); ylabel('e_x (m)');
title(sprintf('X Error, SS mean abs = %.6g m, SS max abs = %.6g m', ss_x_mean, ss_x_max))
legend('Full error','Steady-state window','Location','best')

subplot(3,1,2)
plot(time,y_error,'LineWidth',1.2); hold on; grid on
plot(time(idx_ss),y_error(idx_ss),'r','LineWidth',1.4);
xlabel('Time (s)'); ylabel('e_y (m)');
title(sprintf('Y Error, SS mean abs = %.6g m, SS max abs = %.6g m', ss_y_mean, ss_y_max))
legend('Full error','Steady-state window','Location','best')

subplot(3,1,3)
plot(time,z_error,'LineWidth',1.2); hold on; grid on
plot(time(idx_ss),z_error(idx_ss),'r','LineWidth',1.4);
xlabel('Time (s)'); ylabel('e_z (m)');
title(sprintf('Z Error, SS mean abs = %.6g m, SS max abs = %.6g m', ss_z_mean, ss_z_max))
legend('Full error','Steady-state window','Location','best')

%% Figure 12: Paper-style commanded wind disturbance moments
% This figure reconstructs the commanded disturbance profile from distMode
% and distFactor.  It is independent of the To Workspace logging block, so
% it still shows the requested paper-style disturbance even if out.disturb
% is not connected or has fewer than three channels.
[d_roll_cmd, d_pitch_cmd, d_yaw_cmd] = localPaperWindMoments(time, distMode, distFactor);

WindDisturbanceCommand_Table = table(time, d_roll_cmd, d_pitch_cmd, d_yaw_cmd, ...
    'VariableNames', {'Time_s','d_roll_Nm','d_pitch_Nm','d_yaw_Nm'});
WindDisturbanceConfig_Table = table(distMode, distFactor, ...
    'VariableNames', {'distMode','distFactor'});
assignin('base','WindDisturbanceCommand_Table',WindDisturbanceCommand_Table);
assignin('base','WindDisturbanceConfig_Table',WindDisturbanceConfig_Table);
assignin('base','d_roll_cmd',d_roll_cmd);
assignin('base','d_pitch_cmd',d_pitch_cmd);
assignin('base','d_yaw_cmd',d_yaw_cmd);

figure('Name','Paper-Style Wind Disturbance Commands','Color','w');
subplot(3,1,1)
plot(time, d_roll_cmd, 'LineWidth', 1.2); grid on
xlabel('Time (s)'); ylabel('d_\phi (N.m)');
title('Commanded Roll Disturbance')

subplot(3,1,2)
plot(time, d_pitch_cmd, 'LineWidth', 1.2); grid on
xlabel('Time (s)'); ylabel('d_\theta (N.m)');
title('Commanded Pitch Disturbance')

subplot(3,1,3)
plot(time, d_yaw_cmd, 'LineWidth', 1.2); grid on
xlabel('Time (s)'); ylabel('d_\psi (N.m)');
title('Commanded Yaw Disturbance')

%% Figure 13: Disturbance signals from Simulink log plus commanded reference
% The logged signal is expected to be out.disturb = [d_roll d_pitch d_yaw].
% If your Mux still has only two inputs, the yaw subplot will show only the
% commanded disturbance.  To log yaw, connect windDisturb6 to the third Mux
% input of the To Workspace block named disturb.
hasLoggedDisturb = localSignalExists(out,'disturb');

if hasLoggedDisturb
    disturbData = localGetSignalData(out,'disturb',n);
    disturbData = disturbData(1:min(n,size(disturbData,1)),:);
    timeDist = time(1:size(disturbData,1));
else
    disturbData = [];
    timeDist = time;
end

figure('Name','Disturbance Signals','Color','w');

subplot(3,1,1)
if ~isempty(disturbData) && size(disturbData,2) >= 1
    plot(timeDist, disturbData(:,1), 'b', 'LineWidth', 1.2); hold on
    plot(time, d_roll_cmd, 'r--', 'LineWidth', 1.0);
    legend('Logged disturbance','Commanded disturbance','Location','best')
else
    plot(time, d_roll_cmd, 'LineWidth', 1.2);
end
grid on
xlabel('Time (s)'); ylabel('d_\phi (N.m)');
title('Disturbance in Roll Direction')

subplot(3,1,2)
if ~isempty(disturbData) && size(disturbData,2) >= 2
    plot(timeDist, disturbData(:,2), 'b', 'LineWidth', 1.2); hold on
    plot(time, d_pitch_cmd, 'r--', 'LineWidth', 1.0);
    legend('Logged disturbance','Commanded disturbance','Location','best')
else
    plot(time, d_pitch_cmd, 'LineWidth', 1.2);
end
grid on
xlabel('Time (s)'); ylabel('d_\theta (N.m)');
title('Disturbance in Pitch Direction')

subplot(3,1,3)
if ~isempty(disturbData) && size(disturbData,2) >= 3
    plot(timeDist, disturbData(:,3), 'b', 'LineWidth', 1.2); hold on
    plot(time, d_yaw_cmd, 'r--', 'LineWidth', 1.0);
    legend('Logged disturbance','Commanded disturbance','Location','best')
else
    plot(time, d_yaw_cmd, 'LineWidth', 1.2);
end
grid on
xlabel('Time (s)'); ylabel('d_\psi (N.m)');
title('Disturbance in Yaw Direction')

if localSignalExists(out,'k_adptv_phi') || localSignalExists(out,'k_adptv_theta')
    figure('Name','Adaptive NTSMC Gains','Color','w');
    subplot(2,1,1)
    if localSignalExists(out,'k_adptv_phi')
        k_phi = localGetSignalData(out,'k_adptv_phi',n);
        plot(time(1:min(n,numel(k_phi))), k_phi(1:min(n,numel(k_phi))), 'LineWidth',1.2); grid on
        xlabel('Time (s)'); ylabel('k_\phi'); title('Adaptive Gain for Roll')
    else
        title('Adaptive Gain for Roll not found')
    end

    subplot(2,1,2)
    if localSignalExists(out,'k_adptv_theta')
        k_theta = localGetSignalData(out,'k_adptv_theta',n);
        plot(time(1:min(n,numel(k_theta))), k_theta(1:min(n,numel(k_theta))), 'LineWidth',1.2); grid on
        xlabel('Time (s)'); ylabel('k_\theta'); title('Adaptive Gain for Pitch')
    else
        title('Adaptive Gain for Pitch not found')
    end
end

%% Local helper functions
function time = localGetTime(simOut)
    time = [];
    try
        time = simOut.tout;
    catch
        try
            time = simOut.get('tout');
        catch
        end
    end

    if isempty(time)
        error('Could not find simulation time vector out.tout. Enable time saving in Simulink.');
    end
end

function data = localGetSignalData(simOut, signalName, timeLength)
    raw = [];

    try
        raw = simOut.(signalName);
    catch
        try
            raw = simOut.get(signalName);
        catch
        end
    end

    if isempty(raw)
        error('Signal "%s" was not found in the simulation output.', signalName);
    end

    if isa(raw,'timeseries')
        data = raw.Data;
    elseif isstruct(raw)
        if isfield(raw,'Data')
            data = raw.Data;
        elseif isfield(raw,'signals')
            if numel(raw.signals) == 1
                data = raw.signals.values;
            else
                data = raw.signals(1).values;
            end
        elseif isfield(raw,'Values')
            data = raw.Values.Data;
        else
            error('Unsupported structure format for signal "%s".', signalName);
        end
    elseif isnumeric(raw)
        data = raw;
    elseif isobject(raw) && isprop(raw,'Data')
        data = raw.Data;
    else
        try
            data = raw.Data;
        catch
            error('Unsupported data format for signal "%s".', signalName);
        end
    end

    data = squeeze(data);
    if isvector(data)
        data = data(:);
    end

    % If signal is transposed as channels x time, rotate it to time x channels.
    if nargin >= 3 && ~isempty(timeLength) && size(data,1) ~= timeLength && size(data,2) == timeLength
        data = data.';
    end
end

function tf = localSignalExists(simOut, signalName)
    try
        localGetSignalData(simOut, signalName, []);
        tf = true;
    catch
        tf = false;
    end
end

function err = localAngleError(ref, actual)
    % Wrapped angular error in radians, avoids jumps near +/-pi.
    err = atan2(sin(ref - actual), cos(ref - actual));
end


function [d_roll_cmd, d_pitch_cmd, d_yaw_cmd] = localPaperWindMoments(time, distMode, distFactor)
    % Reconstruct the same paper-style no-model-change wind disturbance used
    % inside the disturbance MATLAB Function block.
    %
    % Outputs:
    %   d_roll_cmd, d_pitch_cmd, d_yaw_cmd are moment disturbances in N.m.

    mode = round(distMode);
    if mode < 0
        mode = 0;
    end

    factor = abs(distFactor);

    % Paper-style timing and normalized amplitudes
    stepAmpPaper = 1.5;
    sineAmpPaper = 1.0;
    sineFreqHz = 0.5;
    sineOmega = 2*pi*sineFreqHz;

    t1 = 6.0;
    t2 = 10.0;
    t3 = 15.0;
    gustWidth = 0.80;

    sineStart = 20.0;
    sineEnd = 30.0;

    % Same moment scaling as the MATLAB Function block.
    if mode == 0
        M_step_scale = 0.0;
        M_sine_scale = 0.0;
    elseif mode == 1
        M_step_scale = 0.0012;
        M_sine_scale = 0.0008;
    elseif mode == 2
        M_step_scale = 0.0030;
        M_sine_scale = 0.0020;
    else
        M_step_scale = 0.0020;
        M_sine_scale = 0.0012;
    end

    time = time(:);
    d_roll_cmd  = zeros(size(time));
    d_pitch_cmd = zeros(size(time));
    d_yaw_cmd   = zeros(size(time));

    for k = 1:numel(time)
        tk = time(k);

        % Step/gust pulses at 6 s, 10 s, and 15 s.
        g1 = 0.0;
        g2 = 0.0;
        g3 = 0.0;

        if tk >= t1 && tk < (t1 + gustWidth)
            g1 = stepAmpPaper;
        end
        if tk >= t2 && tk < (t2 + gustWidth)
            g2 = stepAmpPaper;
        end
        if tk >= t3 && tk < (t3 + gustWidth)
            g3 = stepAmpPaper;
        end

        % 0.5 Hz sinusoidal wind from 20 s to 30 s.
        s = 0.0;
        s90 = 0.0;
        sSlow = 0.0;

        if tk >= sineStart && tk <= sineEnd
            tau = tk - sineStart;
            s = sineAmpPaper * sin(sineOmega*tau);
            s90 = sineAmpPaper * sin(sineOmega*tau + pi/2);
            sSlow = sineAmpPaper * sin(0.70*sineOmega*tau + pi/4);
        end

        d_roll_cmd(k)  = factor * (M_step_scale*( g1 - 0.40*g2) + M_sine_scale*s);
        d_pitch_cmd(k) = factor * (M_step_scale*(-0.50*g1 + g2) + M_sine_scale*s90);
        d_yaw_cmd(k)   = factor * (M_step_scale*( 0.50*g3) + 0.40*M_sine_scale*sSlow);
    end
end


function value = localITAE(time, err)
    % Raw ITAE = integral(t_relative * abs(error) dt).
    % Output unit is error-unit*s^2, for example m*s^2 or rad*s^2.
    time = time(:);
    err = err(:);
    m = min(numel(time), numel(err));
    time = time(1:m);
    err = err(1:m);
    valid = ~(isnan(time) | isnan(err));
    time = time(valid);
    err = err(valid);
    if isempty(err)
        value = NaN;
    else
        t_rel = time - time(1);
        value = trapz(time, t_rel .* abs(err));
    end
end

function value = localITAEEquivalentError(time, err)
    % Convert raw ITAE to an error-unit metric by normalizing with
    % integral(t_relative dt). Output units match err: m for position and
    % rad for attitude.
    time = time(:);
    err = err(:);
    m = min(numel(time), numel(err));
    time = time(1:m);
    err = err(1:m);
    valid = ~(isnan(time) | isnan(err));
    time = time(valid);
    err = err(valid);
    if isempty(err)
        value = NaN;
    else
        t_rel = time - time(1);
        denom = trapz(time, t_rel);
        if denom <= eps
            value = mean(abs(err));
        else
            value = trapz(time, t_rel .* abs(err)) / denom;
        end
    end
end

function value = localRMSE(err)
    err = err(:);
    err = err(~isnan(err));
    if isempty(err)
        value = NaN;
    else
        value = sqrt(mean(err.^2));
    end
end
