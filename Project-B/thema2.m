% to run each method uncomment the respective sections: 
% f(xk+gkdk) MINIMIZATION METHOD,CONSTANT GAMMA MINIMIZATION METHOD,
% ARMIJO METHOD 

clear all;

e=10^(-3);
x(1)=1;
y(1)=1;

% Define the function
f = @(x, y) x.^3 .* exp(-x.^2 - y.^4);

df_dx= @(x, y) -x^2*(2*x^2-3)*exp(-x^2-y^4);
df_dy= @(x, y) -4*x^3*y^3*exp(-x^2-y^4);

% Create a meshgrid for contour plot
[x_mesh, y_mesh] = meshgrid(-3:0.1:0, -3:0.1:3);
z = f(x_mesh, y_mesh);

figure(1);
% Plot contour lines
contour(x_mesh, y_mesh, z, 50, 'LineWidth', 1.5);
hold on;

grad=[df_dx(x(1),y(1)); df_dy(x(1),y(1))];

k=1;


% f(xk+gkdk) MINIMIZATION METHOD
%{
while norm(grad)>e
    %fprintf('%d \n',norm(grad));
    
    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
    
    d(k, :) = -grad;
    %fprintf('[%d, %d ] \n',d(k,1),d(k,2));
    
    objective_function = @(gamma) f(x(k) + gamma * d(k, 1), y(k) + gamma * d(k, 2));
    
    % Search for gamma that minimizes the objective function
    optimal_gamma = gamma_opti(objective_function,0,3);
    %optimal_gamma = fminbnd(objective_function, 0, 5);
    fprintf('optimal=%d \n',optimal_gamma);
    
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

% Create a grid of x and y values
[p, g] = meshgrid(-3:0.1:3, -3:0.1:3);

% Evaluate the function on the grid
z = f(p, g);

% Plot the surface
figure(2);
%figure;
fsurf(f, [-3 3 -3 3]);
colorbar

hold on;
x_highlight = x(end);
y_highlight = y(end);
z_highlight = f(x(end),y(end));
scatter3(x_highlight, y_highlight, z_highlight, 100, 'r', 'filled'); % Red marker, replace 100 with your desired marker size
hold off;

title("3D Plot of f(x,y)");
xlabel('x');
ylabel('y');
zlabel('f(x, y)');


store_k(k) = k;
store_f(k) = f(x(k),y(k));
hold on;
figure(3);
plot(store_k,store_f,'-s')
xlabel('k')
ylabel('f(x,y)')
title("Iterations");
hold off;
%}


% CONSTANT GAMMA MINIMIZATION METHOD
%{
gamma=1;
while norm(grad)>e
    fprintf('%d \n',norm(grad));

    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
    
    d(k, :) = -grad;
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + gamma * d(k, 1);
    y(k+1) = y(k) + gamma * d(k, 2);

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

% Create a grid of x and y values
[p, g] = meshgrid(-3:0.1:3, -3:0.1:3);

% Evaluate the function on the grid
z = f(p, g);

% Plot the surface
figure(2);
%figure;
fsurf(f, [-3 3 -3 3]);
colorbar

hold on;
x_highlight = x(end);
y_highlight = y(end);
z_highlight = f(x(end),y(end));
scatter3(x_highlight, y_highlight, z_highlight, 100, 'r', 'filled'); % Red marker, replace 100 with your desired marker size
hold off;

title("3D Plot of f(x,y)");
xlabel('x');
ylabel('y');
zlabel('f(x, y)');


store_k(k) = k;
store_f(k) = f(x(k),y(k));
hold on;
figure(3);
plot(store_k,store_f,'-s')
xlabel('k')
ylabel('f(x,y)')
title("Iterations");
hold off;
%}



% ARMIJO METHOD
alpha = 1e-5; % You can adjust alpha as needed
beta = 0.5;  % You can adjust beta as needed
s = 0.5;
gamma=s;
mk = 0;
while norm(grad)>e
    fprintf('%d \n',norm(grad));

    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
    
    d(k, :) = -grad;
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + gamma * d(k, 1);
    y(k+1) = y(k) + gamma * d(k, 2);

     % Plot the current point
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;

    
    % Armijo condition
    while f(x(k), y(k))-f(x(k+1), y(k+1)) < -alpha*beta^(mk)*s*d(k).'*grad
        mk=mk+1;
    end
    gamma = s*beta^(mk);
    
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k+1), y(k+1)); df_dy(x(k+1), y(k+1))];
    
    k = k + 1;
    
end


xlabel('x');
ylabel('y');
title('Optimization Path');
grid on;

% Create a grid of x and y values
[p, g] = meshgrid(-3:0.1:3, -3:0.1:3);

% Evaluate the function on the grid
z = f(p, g);

% Plot the surface
figure(2);
%figure;
fsurf(f, [-3 3 -3 3]);
colorbar

hold on;
x_highlight = x(end);
y_highlight = y(end);
z_highlight = f(x(end),y(end));
scatter3(x_highlight, y_highlight, z_highlight, 100, 'r', 'filled'); % Red marker, replace 100 with your desired marker size
hold off;

title("3D Plot of f(x,y)");
xlabel('x');
ylabel('y');
zlabel('f(x, y)');


store_k(k) = k;
store_f(k) = f(x(k),y(k));
hold on;
figure(3);
plot(store_k,store_f,'-s')
xlabel('k')
ylabel('f(x,y)')
title("Iterations");
hold off;



fprintf('x=%d \n',x(end));
fprintf('y=%d \n',y(end));
fprintf('k=%d \n',k);