function GenerateDiscretizedModels(Q, R) 
% GenerateDiscretizedModels - Builds continuous-time models, discretizes them,
% and exports discrete models for ALL road presets to the Simulink workspace.
%
% Inputs (optional):
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
%   Exports A_d, B_d, C_d, D_d for *each road condition* 
%   (e.g. A_d_dry_tarmac, A_d_wet_tarmac, etc.) to the Simulink base workspace.

    % --- Road Condition Presets ---
    presets.dry_tarmac = [10, 1.9, 1.0, 0.97];
    presets.wet_tarmac = [12, 2.3, 0.82, 1.0];
    presets.snow       = [5,  2.0, 0.3,  1.0];
    presets.ice        = [4,  2.0, 0.1,  1.0];

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

    % --- Discretization Parameters ---
    Ts = 0.01;  % [s] Sampling time

    % --- Loop through all presets ---
    presetNames = fieldnames(presets);

    stateSpaces = {};

    for i = 1:length(presetNames)
        presetName = presetNames{i};
        coeffs = presets.(presetName);

        % --- Preset Coefficients ---
        [B, C, D, E] = deal(coeffs(1), coeffs(2), coeffs(3), coeffs(4));
        Clambda = LinearizedMagicFormulaCalculator(D, C, B, E);

        % --- Continuous-Time System Matrices ---
        A = [
            (-4 * Clambda * gravitationalAcceleration * cos(roadSlope)) / wheelRadius - C1 / vehicleMass,  4 * Clambda * gravitationalAcceleration * cos(roadSlope),  0;
            (4 * Clambda * gravitationalAcceleration * cos(roadSlope)) / (motorInertia + wheelInertia), ...
            - (4 * Clambda * wheelRadius * vehicleMass * gravitationalAcceleration * cos(roadSlope)) / (motorInertia + wheelInertia), ...
            motorTorqueConstant / (motorInertia + wheelInertia);
            0, -motorBackEMFConstant / motorInductance, -motorResistance / motorInductance
        ];

        Bmat = [0; 0; 1 / motorInductance];
        Cmat = eye(3);
        Dmat = zeros(3,1);

        % --- Discretization ---
        sys_c = ss(A, Bmat, Cmat, Dmat);
        sys_d = c2d(sys_c, Ts, 'zoh');
    
        % Store matrices
        stateSpace.(presetName).A = sys_d.A;
        stateSpace.(presetName).B = sys_d.B;
        stateSpace.(presetName).C = sys_d.C;
        stateSpace.(presetName).D = sys_d.D;

        % --- Export with preset-specific names ---
        assignin('base', "stateSpace", stateSpace);
    end

    % Export common variables
    assignin('base', 'Ts', Ts);

end
