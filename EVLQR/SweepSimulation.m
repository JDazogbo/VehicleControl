% --- Configuration --- %
roadConditions = {'dry_tarmac', 'wet_tarmac', 'snow', 'ice'};
velocityWeights = linspace(1, 1000, 30);  % Expanded sweep range
targetError = 200;
tolerance = 100;  % Acceptable deviation from targetError
maxIterations = 5;

% --- Preallocate Results --- %
results = struct();

% --- Loop Over Road Conditions --- %
for rcIdx = 1:length(roadConditions)
    selectedRoad = roadConditions{rcIdx};
    fprintf('\n================ Evaluating Road Condition: %s ================\n', selectedRoad);

    % Loop over terrain estimation modes (1 to 5)
    for terrainEstimate = 1:5
        fprintf('\n-- Terrain Estimate: %d --\n', terrainEstimate);

        % Set dummy constant in base workspace
        assignin('base', 'TerrainEstimatorMode', terrainEstimate);

        bestError = inf;
        bestConfig = struct();

        for wIdx = 1:length(velocityWeights)
            velWeight = velocityWeights(wIdx);
            fprintf('  → Trying velocity weight: %.2f\n', velWeight);

            % Call your init function with parameters
            GenerateLQRGains(selectedRoad, diag([velWeight, 1e-12, 1e-6]), 0.01);

            % Run simulation
            Res = sim('VehicleEnergyManagementSystem', 'StopTime', '1000', 'AlgebraicLoopMsg', 'off');
            velocityError = Res.logsout.get('Velocity Error').Values.Data(end);
            energyUsed = Res.logsout.get('Energy Expenditure (KJ)').Values.Data(end);
            fprintf('Velocity Error=%.2f\n', velocityError);
            fprintf('Energy Expenditure=%.2f\n', energyUsed);
            % Save result
            key = sprintf('TerrainEstimate_%d', terrainEstimate);
            results.(selectedRoad).(key)(wIdx).VelocityWeight = velWeight;
            results.(selectedRoad).(key)(wIdx).VelocityError = velocityError;
            results.(selectedRoad).(key)(wIdx).EnergyConsumption = energyUsed;

            % Check if best so far
            if abs(velocityError - targetError) < abs(bestError - targetError)
                bestError = velocityError;
                bestConfig = struct('VelocityWeight', velWeight, ...
                                    'VelocityError', velocityError, ...
                                    'EnergyConsumption', energyUsed);
            end

            % Early stopping if within tolerance
            if abs(velocityError - targetError) <= tolerance
                fprintf('  ✓ Acceptable config found (%.2f error).\n', velocityError);
                break;
            end
        end

        % Display best result for this terrain estimate
        fprintf('  ✅ Best for %s @ Estimate %d: Weight = %.1f, Error = %.2f, Energy = %.2f\n', ...
            selectedRoad, terrainEstimate, bestConfig.VelocityWeight, bestConfig.VelocityError, bestConfig.EnergyConsumption);
    end
end

% Save all results
save('lqr_sweep_results.mat', 'results');
