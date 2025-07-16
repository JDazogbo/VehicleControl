warning('off');

% Controller parameters
terrainType = 'dry_tarmac';
Q = diag([100, 1e-12, 1e-6]); % [Velocity Penalty, Angular Velocity Penalty, Current Penalty]
R = 1e-4; % Input Penalty


% Call the gain generator with these parameters

% Select the terrain estimator
%    1 - Dry Tarmac
%    2 - Wet Tarmac
%    3 - Snow
%    4 - Ice
%    5 - Adaptive Estimator
terrainEstimator = 5;

GenerateLQRGains(terrainType, Q, R, terrainEstimator);


% Set the scope title using the same terrain and estimator info
% SetScopeTitle(terrainType, terrainEstimator);
