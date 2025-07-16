function LQRGains = GenerateLQRGains(selectedPreset, Q, R, terrainEstimate)
% generateLQRGains - Computes LQR gains for all road presets and exports to Simulink workspace.
%
% Inputs (optional):
%   selectedPreset (string) - Road preset to highlight (e.g. 'dry_tarmac')
%   Q (3x3 matrix)           - LQR state cost matrix
%                              -> [Velocity Penalty, Angular Velocity Penalty, Current Penalty]
%   R (scalar)               - LQR input cost
%   terrainEstimate          - Selected Road Condition preset for the
%                              underlying controller model. 
%                              -> 1. Dry Tarmac
%                              -> 2. Wet Tarmac
%                              -> 3. Snow
%                              -> 4. Ice
%                              -> 5. Adaptive Estimator
%
% Output:
%   LQRGains (4x3 matrix)    - Each row: [GainVf, GainWf, GainIf]

    % --- Road Condition Presets ---
    presets.dry_tarmac = [10, 1.9, 1.0, 0.97];
    presets.wet_tarmac = [12, 2.3, 0.82, 1.0];
    presets.snow       = [5,  2.0, 0.3, 1.0];
    presets.ice        = [4,  2.0, 0.1, 1.0];

    % Confirm selected preset exists
    if isfield(presets, selectedPreset)
        coeffs = presets.(selectedPreset);

    else
        error('❌ Unknown road friction preset: "%s"\nAvailable options: %s', ...
            selectedPreset, strjoin(fieldnames(presets), ', '));
    end

    % --- Constants ---
    gravitationalAcceleration = 9.81;
    roadSlope = 0;
    vehicleMass = 1500;
    wheelInertia = 0.8;
    wheelRadius = 0.3;
    motorInertia = 1;
    motorResistance = 0.1;
    motorInductance = 0.005;
    motorTorqueConstant = 5;
    motorBackEMFConstant = 0.004;
    C1 = 3.87;

    % --- Loop through presets ---
    presetNames = fieldnames(presets);
    nPresets = numel(presetNames);
    LQRGains = zeros(nPresets, 3);  % [Vf Wf If] per preset

    for i = 1:nPresets
        name = presetNames{i};
        coeffs_i = presets.(name);
        [B, C, D, E] = deal(coeffs_i(1), coeffs_i(2), coeffs_i(3), coeffs_i(4));

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
    end

    % Export to Simulink base workspace
    assignin('base', 'LQRGains', LQRGains);
    assignin('base', 'SelectedRoadCoefficients', coeffs);
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
    assignin('base', "terrainEstimate", terrainEstimate);
    assignin('base', 'C1', C1);
    assignin('base', 'StateWeightMatrix', Q);
    assignin('base', 'InputWeightMatrix', R)

end
