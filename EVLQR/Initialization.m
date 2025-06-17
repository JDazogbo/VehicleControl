fprintf(['\n\n----------------------Simulation Initialisation Script----------------------\n\n' ...
    '']);

% --- Road Condition Presets ---
roadPresets = {
    'Dry Tarmac', [10, 1.9, 1.0, 0.97];
    'Wet Tarmac', [12, 2.3, 0.82, 1.0];
    'Snow',       [5,  2.0, 0.3, 1.0];
    'Ice',        [4,  2.0, 0.1, 1.0];
};

nPresets = size(roadPresets, 1);

% --- SELECT SIMULATION ROAD FRICTION PRESET ---
% Choose one of the following: 'dry_tarmac', 'wet_tarmac', 'snow', 'ice'
selectedPreset = 'dry_tarmac';

% Define road friction presets: [B, C, D, E]
presets.dry_tarmac = [10, 1.9, 1.0, 0.97];
presets.wet_tarmac = [12, 2.3, 0.82, 1.0];
presets.snow       = [5,  2.0, 0.3, 1.0];
presets.ice        = [4,  2.0, 0.1, 1.0];

% Apply selected preset
if isfield(presets, selectedPreset)
    coeffs = presets.(selectedPreset);
    roadB = coeffs(1);
    roadC = coeffs(2);
    roadD = coeffs(3);
    roadE = coeffs(4);
    fprintf('✔ Using "%s" preset: B=%.2f, C=%.2f, D=%.2f, E=%.2f\n\n', ...
        selectedPreset, roadB, roadC, roadD, roadE);
else
    error('❌ Unknown road friction preset: "%s"\nAvailable options: %s', ...
        selectedPreset, strjoin(fieldnames(presets), ', '));
end

% --- Constants ---
gravitationalAcceleration = 9.81;             % Gravity
roadSlope = 0;        % Radians
vehicleMass = 1500;   % kg
wheelInertia = 0.8;   % kg·m^2
wheelRadius = 0.3;    % m
motorInertia = 1;     % kg·m^2
motorResistance = 0.1; % Ohms
motorInductance = 0.005; % H
motorTorqueConstant = 5; % Nm/A
motorBackEMFConstant = 0.004; % V⋅s/rad

% Coast Down Coefficients
C0 = 194.87; % Constant resistance
C1 = 3.87;   % Rolling resistance coefficient
C2 = 0.37;   % Aerodynamic drag coefficient


% --- LQR Weights ---
Q = diag([100, 1e-12, 1e-6]);  % State penalties - Vehicle Velocity, Angular Velocity, Armature Current
R = 10;                     % Input penalty

% --- Preallocate Gain Matrix ---
% Row = preset index, Col = [Vf Wf If]
LQRGains = zeros(nPresets, 3);

% --- Loop Through Each Road Condition ---
for i = 1:nPresets
    presetName = roadPresets{i,1};
    coeffs = roadPresets{i,2};
    [B, C, D, E] = deal(coeffs(1), coeffs(2), coeffs(3), coeffs(4));

    Clambda = LinearizedMagicFormulaCalculator(D, C, B, E);

    A = [
        (-4 * Clambda * g * cos(roadSlope)) / wheelRadius - C1 / vehicleMass,  4 * Clambda * g * cos(roadSlope),  0;
        (4 * Clambda * g * cos(roadSlope)) / (motorInertia + wheelInertia), ...
        - (4 * Clambda * wheelRadius * vehicleMass * g * cos(roadSlope)) / (motorInertia + wheelInertia), ...
        motorTorqueConstant / (motorInertia + wheelInertia);
        0, -motorBackEMFConstant / motorInductance, -motorResistance / motorInductance
    ];

    Bmat = [0; 0; 1 / motorInductance];

    % Compute LQR gain
    K = lqr(A, Bmat, Q, R);

    % Store
    LQRGains(i, :) = K;

    fprintf('✔ %s LQR gains: [%.6f, %.6f, %.6f]\n', presetName, K(1), K(2), K(3));
end

% Export to Simulink Workspace
assignin('base', 'LQRGains', LQRGains);