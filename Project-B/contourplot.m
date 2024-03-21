clear all;

e = 1e-3;
x(1) = -1;
y(1) = -1;

% Define the function
f = @(x, y) x.^3 .* exp(-x.^2 - y.^4);

df_dx = @(x, y) -x^2*(2*x^2-3)*exp(-x^2-y^4);
df_dy = @(x, y) -4*x^3*y^3*exp(-x^2-y^4);


% Create a meshgrid for contour plot
[x_mesh, y_mesh] = meshgrid(-3:0.1:0, -3:0.1:3);
z = f(x_mesh, y_mesh);

% Plot contour lines
contour(x_mesh, y_mesh, z, 50, 'LineWidth', 1.5);
hold on;

grad = [df_dx(x(1),y(1)); df_dy(x(1),y(1))];

k = 1;
while norm(grad) > e
    d(k, :) = -grad;
    
    objective_function = @(gamma) f(x(k) + gamma * d(k, 1), y(k) + gamma * d(k, 2));
    
    % Search for gamma that minimizes the objective function
    optimal_gamma = fminbnd(objective_function, 0, 3);
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + optimal_gamma * d(k, 1);
    y(k+1) = y(k) + optimal_gamma * d(k, 2);
    
    % Plot the current point
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k+1), y(k+1)); df_dy(x(k+1), y(k+1))];
    
    k = k + 1;
end

xlabel('x');
ylabel('y');
title('Optimization Path');
grid on;
