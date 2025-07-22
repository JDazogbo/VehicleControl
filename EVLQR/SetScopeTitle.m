function SetScopeTitle(terrainType, estimatorID)
    % Sets the main title of the Time Scope block with terrain and estimator info
    %
    % terrainType: string, e.g. 'dry tarmac'
    % estimatorID: integer from 1 to 5 selecting estimator name
    
    scopeBlock = 'VehicleEnergyManagementSystem/Velocity';

    % Estimator mapping
    estimatorNames = { ...
        'Dry Tarmac', ...
        'Wet Tarmac', ...
        'Snow', ...
        'Ice', ...
        'Adaptive Estimator'};

    % Validate estimatorID
    if estimatorID < 1 || estimatorID > numel(estimatorNames)
        error('Estimator ID must be between 1 and %d', numel(estimatorNames));
    end

    % Compose title string
    titleString = sprintf('Drive Cycle (m/s) | Terrain type: %s | Estimator: %s', ...
        terrainType, estimatorNames{estimatorID});

    % Set the main scope title
    blockHandle = get_param(scopeBlock, 'Handle');
    scopeConfig = get_param(blockHandle, 'ScopeConfiguration');
    scopeConfig.Title = titleString;
end
