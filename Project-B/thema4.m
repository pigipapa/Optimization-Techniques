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
d2f_dx2 = @(x, y) (4.*x.^5-14.*x.^3+6.*x).*exp(-x.^2-y.^4);
d2f_dxdy = @(x, y) (8.*x.^4-12.*x.^2).*y.^3.*exp(-x.^2-y.^4);
d2f_dy2 = @(x, y) 4.*x.^3*y.^2.*(4.*y.^4-3).*exp(-x.^2-y.^4);
d2f_dydx = @(x, y) 4*x^2*y^3*(2*x^2-3)*exp(-x^2-y^4); 


criteria_check = true;


% Create a meshgrid for contour plot
[x_mesh, y_mesh] = meshgrid(-3:0.1:0, -3:0.1:3);
z = f(x_mesh, y_mesh);

figure(1);
% Plot contour lines
contour(x_mesh, y_mesh, z, 50, 'LineWidth', 1.5);
hold on;


% f(xk+gkdk) MINIMIZATION METHOD
%{
mk=0;
k=1;
while true
    
    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k), y(k)); df_dy(x(k), y(k))];
    hess=[d2f_dx2(x(k),y(k)),d2f_dxdy(x(k),y(k));d2f_dydx(x(k),y(k)),d2f_dy2(x(k),y(k))];
   
    if round(hess(1,2),4)~=round(hess(2,1),4)
       disp('The hessian is not symmetric!');
       break;
    end

    while any(eig(hess+mk*eye(size(hess))) <= 0)
        mk=mk+0.3;  
    end
    fprintf('mk=%d \n', mk);
    
    
    % Solve for dk
    dk = -(hess + mk * eye(size(hess))) \ grad;
    
    objective_function = @(gamma) f(x(k) + gamma * dk(1), y(k) + gamma * dk(2));
    
    optimal_gamma = gamma_opti(objective_function,0,3);
    %optimal_gamma = fminbnd(objective_function, 0, 5);
    %fprintf('optimal=%d \n',optimal_gamma);
    
    %Update x and y based on the optimal gamma
    x(k+1) = x(k) + optimal_gamma * dk(1);
    y(k+1) = y(k) + optimal_gamma * dk(2);

    % Plot the current point
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;

    criteria_check = criteria(f, df_dx, df_dy, grad, x, y, optimal_gamma, dk);
            
    if ~criteria_check
        fprintf('BREAK IN CRITERIA!');
          break;
    end
    
     % Check convergence
    if norm(grad) < e
        break;
    end
    fprintf('norm=%d \n', norm(grad));
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
%}



% CONSTANT GAMMA MINIMIZATION METHOD
mk=0;
k=1;
optimal_gamma = 0.065;
while true
    
    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
     
    % Compute the gradient at the new point
    grad = [df_dx(x(k), y(k)); df_dy(x(k), y(k))];
    hess=[d2f_dx2(x(k),y(k)),d2f_dxdy(x(k),y(k));d2f_dydx(x(k),y(k)),d2f_dy2(x(k),y(k))];
    
    if round(hess(1,2),4)~=round(hess(2,1),4)
       disp('The hessian is not symmetric!');
       break;
    end
  
    while any(eig(hess+mk*eye(size(hess))) <= 0)   
        mk=mk+0.1;  
    end
    fprintf('mk=%d \n', mk);
    
    % Solve for dk
    dk = -(hess + mk * eye(size(hess))) \ grad;
    
    
    
    %Update x and y based on the optimal gamma
    x(k+1) = x(k) + optimal_gamma * dk(1);
    y(k+1) = y(k) + optimal_gamma * dk(2);
    
    % Plot the current point
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;

    criteria_check = criteria(f, df_dx, df_dy, grad, x, y, optimal_gamma, dk);
            
    if ~criteria_check
        fprintf('BREAK IN CRITERIA!');
          break;
    end
    
     % Check convergence
    if norm(grad) < e
        break;
    end
    fprintf('norm=%d \n', norm(grad));
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


% ARMIJO METHOD
%{
alpha = 1e-3; % You can adjust alpha as needed
beta = 0.2;  % You can adjust beta as needed
s = 0.35;
gamma=s;
m=0;
mk=0;
k=1;
while true
     
    store_k(k) = k;
    store_f(k) = f(x(k),y(k));

    % Compute the gradient at the new point
    grad = [df_dx(x(k), y(k)); df_dy(x(k), y(k))];
    hess=[d2f_dx2(x(k),y(k)),d2f_dxdy(x(k),y(k));d2f_dydx(x(k),y(k)),d2f_dy2(x(k),y(k))];
   
    if round(hess(1,2),4)~=round(hess(2,1),4)
       disp('The hessian is not symmetric!');
       break;
    end

    while any(eig(hess+m*eye(size(hess))) <= 0)
        m=m+0.3;  
    end
    fprintf('mk=%d \n', m);
    
    % Solve for dk
    dk = -(hess + m * eye(size(hess))) \ grad;
    
    %Update x and y based on the optimal gamma
    x(k+1) = x(k) + gamma * dk(1);
    y(k+1) = y(k) + gamma * dk(2);

     % Plot the current point
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;
    
    criteria_check = criteria(f, df_dx, df_dy, grad, x, y, gamma, dk);
            
    if ~criteria_check
        fprintf('BREAK IN CRITERIA!');
          break;
    end
    
    % Armijo condition
    while f(x(k), y(k))-f(x(k+1), y(k+1)) < -alpha*beta^(mk)*s*dk.'*grad
        mk=mk+0.1;
    end
    gamma = s*beta^(mk);
    
     % Check convergence
    if norm(grad) < e
        break;
    end
    
    fprintf('norm=%d \n', norm(grad));
    
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
%}