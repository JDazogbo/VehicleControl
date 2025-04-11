function slope = LinearizedMagicFormulaCalculator(D, C, B, E)
    % LinearizedMagicFormulaCalculator - Computes the slope of the Pacejka Magic Formula at 0.
    %
    % Inputs:
    %   D - Peak value (amplitude)
    %   C - Shape factor
    %   B - Stiffness factor
    %   E - Curvature factor
    %
    % Outputs:
    %   slope - The computed slope (derivative) of the formula at zero slip.
    %
    % Usage:
    %   slope = LinearizedMagicFormulaCalculator(D, C, B, E);

    % Define the Pacejka Magic Formula as a function handle
    %pacejkaFormula = @(x) D * sin(C * atan(B * x - E * (B * x - atan(B * x))));
    
    % Linear Approximation of Pacejka's
    % From the MATLAB Documentation, https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html
    
    slope = B*C*D

    % Display the result
    fprintf('The slope of the Pacejka formula at zero is: %.6f\n', slope);
end
