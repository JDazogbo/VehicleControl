function LQRGains = GenerateLQRGains(selectedPreset, Q, R)
% generateLQRGains - Computes LQR gains for 4 road presets and exports to Simulink workspace.
%
% Inputs (optional):
%   selectedPreset (string) - Road preset to highlight (e.g. 'dry_tarmac')
%   Q (3x3 matrix)           - LQR state cost matrix
%   R (scalar)               - LQR input cost
%
% Output:
%   LQRGains (4x3 matrix)    - Each row: [GainVf, GainWf, GainIf]

    % fprintf(['\n\n----------------------Simulation Initialisation Script----------------------\n\n']);

    % --- Defaults if not provided ---
    if nargin < 1 || isempty(selectedPreset)
        selectedPreset = 'dry_tarmac';
    end
    if nargin < 2 || isempty(Q)
        Q = diag([100, 1e-12, 1e-6]);  % Default state weights
    end
    if nargin < 3 || isempty(R)
        R = 0.01;  % Default input weight
    end

    % --- Road Condition Presets ---
    roadPresets = {
        'Dry Tarmac', [10, 1.9, 1.0, 0.97];
        'Wet Tarmac', [12, 2.3, 0.82, 1.0];
        'Snow',       [5,  2.0, 0.3, 1.0];
        'Ice',        [4,  2.0, 0.1, 1.0];
    };

    presets.dry_tarmac = [10, 1.9, 1.0, 0.97];
    presets.wet_tarmac = [12, 2.3, 0.82, 1.0];
    presets.snow       = [5,  2.0, 0.3, 1.0];
    presets.ice        = [4,  2.0, 0.1, 1.0];

    % Confirm selected preset exists
    if isfield(presets, selectedPreset)
        coeffs = presets.(selectedPreset);
        % fprintf('✔ Using "%s" preset: B=%.2f, C=%.2f, D=%.2f, E=%.2f\n\n',selectedPreset, coeffs(1), coeffs(2), coeffs(3), coeffs(4));
    else
        error('❌ Unknown road friction preset: "%s"\nAvailable options: %s', ...
            selectedPreset, strjoin(fieldnames(presets), ', '));
    end

    % --- Constants ---
    gravitationalAcceleration = 9.81;               % Gravity
    roadSlope = 0;          % Radians
    vehicleMass = 1500;     % kg
    wheelInertia = 0.8;     % kg·m^2
    wheelRadius = 0.3;      % m
    motorInertia = 1;       % kg·m^2
    motorResistance = 0.1;  % Ohms
    motorInductance = 0.005; % H
    motorTorqueConstant = 5; % Nm/A
    motorBackEMFConstant = 0.004; % V⋅s/rad
    C1 = 3.87;              % Rolling resistance coefficient

    nPresets = size(roadPresets, 1);
    LQRGains = zeros(nPresets, 3);  % [Vf Wf If] per preset

    for i = 1:nPresets
        presetName = roadPresets{i,1};
        coeffs = roadPresets{i,2};
        [B, C, D, E] = deal(coeffs(1), coeffs(2), coeffs(3), coeffs(4));

        Clambda = LinearizedMagicFormulaCalculator(D, C, B, E);

        A = [
            (-4 * Clambda * gravitationalAcceleration * cos(roadSlope)) / wheelRadius - C1 / vehicleMass,  4 * Clambda * gravitationalAcceleration * cos(roadSlope),  0;
            (4 * Clambda * gravitationalAcceleration * cos(roadSlope)) / (motorInertia + wheelInertia), ...
            - (4 * Clambda * wheelRadius * vehicleMass * gravitationalAcceleration * cos(roadSlope)) / (motorInertia + wheelInertia), ...
            motorTorqueConstant / (motorInertia + wheelInertia);
            0, -motorBackEMFConstant / motorInductance, -motorResistance / motorInductance
        ];

        Bmat = [0; 0; 1 / motorInductance];

        K = lqr(A, Bmat, Q, R);
        LQRGains(i, :) = K;

     %   fprintf('✔ %s LQR gains: [%.6f, %.6f, %.6f]\n', presetName, K(1), K(2), K(3));
    end

    % Export to Simulink base workspace
    assignin('base', 'LQRGains', LQRGains);
    assignin('base', 'SelectedRoadCoefficients', coeffs);
    % Assign constants to base workspace
    assignin('base', 'gravitationalAcceleration', gravitationalAcceleration);
    assignin('base', 'roadSlope', roadSlope);
    assignin('base', 'vehicleMass', vehicleMass);
    assignin('base', 'wheelInertia', wheelInertia);
    assignin('base', 'wheelRadius', wheelRadius);
    assignin('base', 'motorInertia', motorInertia);
    assignin('base', 'motorResistance', motorResistance);
    assignin('base', 'motorInductance', motorInductance);
    assignin('base', 'motorTorqueConstant', motorTorqueConstant);
    assignin('base', 'motorBackEMFConstant', motorBackEMFConstant);
    assignin('base', 'C1', C1);

end