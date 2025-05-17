fprintf(['\n\n----------------------Simulation Initialisation Script----------------------\n\n' ...
    '']);

% --- SELECT ROAD FRICTION PRESET ---
% Choose one of the following: 'dry_tarmac', 'wet_tarmac', 'snow', 'ice'
selectedPreset = 'ice';

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




% --- COMPUTE LQR CONTROLLER GAINS ---
% Environment Constants
gravitationalAcceleration = 9.81; % m/s^2
roadSlope = 0; % Road slope angle (radians)

% Vehicle Parameters
vehicleMass = 1500; % kg
wheelInertia = 0.8; % kg*m^2
wheelRadius = 0.3; % m

% Motor Parameters
motorInertia = 1; % kg*m^2
motorResistance = 0.1; % Ohms
motorInductance = 0.005; % H
motorTorqueConstant = 5; % Nm/A
motorBackEMFConstant = 0.004; % V⋅s/rad

% Coast Down Coefficients
C0 = 194.87; % Constant resistance
C1 = 3.87;   % Rolling resistance coefficient
C2 = 0.37;   % Aerodynamic drag coefficient

% Linearized Pacejka Slope
Clambda = LinearizedMagicFormulaCalculator(roadD, roadC, roadB, roadE);

% State-Space Model Matrices
% Define A matrix incorporating vehicle and motor dynamics
A = [
    (- 4* Clambda * gravitationalAcceleration * cos(roadSlope)) / wheelRadius - C1 / vehicleMass, 4*Clambda * gravitationalAcceleration * cos(roadSlope), 0;
    (4*Clambda * gravitationalAcceleration * cos(roadSlope)) / (motorInertia + wheelInertia), - (4*Clambda * wheelRadius * vehicleMass * gravitationalAcceleration * cos(roadSlope)) / (motorInertia + wheelInertia), motorTorqueConstant / (motorInertia + wheelInertia);
    0, -motorBackEMFConstant / motorInductance, -motorResistance / motorInductance
];

% Define B matrix with input voltage (Vin)
B = [
    0;
    0;
    1 / motorInductance
];

% Output matrix (only velocity is measured)
C = [1 0 0]; 
D = 0; 

% LQR Controller Design
% Define weight matrices
% Penalizes velocity, wheel angular velocity, and armature current deviations
Q = diag([100 0.000000000001, 0.000001]); 
% Penalizes input voltage
R = 0.001; 

% Display the weights for states and input
%disp('LQR Weighting Matrices:');
%disp('State Weights (Q):');
%fprintf('Velocity State Weight: %.2f\n', Q(1,1));
%fprintf('Wheel Angular Velocity State Weight: %.2f\n', Q(2,2));
%fprintf('Armature Current State Weight: %.2f\n', Q(3,3));
%disp('Input Weight (R):');
%fprintf('Voltage Input Weight: %.2f\n', R);

% Compute LQR gain
K = lqr(A, B, Q, R);

% Display the LQR gain
disp('LQR Gain (K):');
disp(K);

% Store the gains for Simulink
GainVf = K(1); % Gain for vehicle velocity state
GainWf = K(2); % Gain for wheel angular velocity state
GainIf = K(3); % Gain for armature current state

% Building the State-Space Model
% Subtract BK from A to incorporate the state feedback
sys = ss(A - B * K, B, C, D);

% Compute the DC gain for scaling the reference input
dc_gain = dcgain(sys);
GainVr = 1; % Scale the reference input to ensure output amplitude is 1

% Plot the step response of the scaled system
disp('Scaled Step Response:');
scaled_sys = GainVr * sys; % Apply the scaled reference signal

% Uncomment to Compute Step Response
% step(scaled_sys);

% Display the Simulink gain variables
disp('Simulink Gain Variables:');
fprintf('GainVr (Reference Velocity Gain) = %.6f\n', GainVr);
fprintf('GainWf (Wheel Angular Velocity Gain) = %.6f\n', GainWf);
fprintf('GainVf (Vehicle Velocity Gain) = %.6f\n', GainVf);
fprintf('GainIf (Armature Current Gain) = %.6f\n', GainIf);

