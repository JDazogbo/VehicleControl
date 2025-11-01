 warning('off');


% Controller parameters
terrainType = 'dry_tarmac';

% Cost function penalties
Q = diag([50, 1e-12, 1e-12]); % [Velocity Penalty, Angular Velocity Penalty, Current Penalty]
R = 1e-6; % Input Penalty

% --- Road Condition Presets ---
presets.dry_tarmac = [10, 1.9, 1.0, 0.97];
presets.wet_tarmac = [12, 2.3, 0.82, 1.0];
presets.snow       = [5,  2.0, 0.3, 1.0];
presets.ice        = [4,  2.0, 0.1, 1.0];

% Confirm selected preset exists
if isfield(presets, terrainType)
    coeffs = presets.(terrainType);
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


% Export to Simulink base workspace
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
assignin('base', 'C1', C1);
assignin('base', 'StateWeightMatrix', Q);
assignin('base', 'InputWeightMatrix', R)

GenerateDiscretizedModels(Q, R)