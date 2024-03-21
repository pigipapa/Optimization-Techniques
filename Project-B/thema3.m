% to run each method uncomment the respective sections: 
% f(xk+gkdk) MINIMIZATION METHOD,CONSTANT GAMMA MINIMIZATION METHOD,
% ARMIJO METHOD 

clear all;

e=10^(-3);
x(1)=0;
y(1)=0;

% Define the function
f = @(x, y) x.^3 .* exp(-x.^2 - y.^4);
df_dx= @(x, y) -x^2*(2*x^2-3)*exp(-x^2-y^4);
df_dy= @(x, y) -4*x^3*y^3*exp(-x^2-y^4);
d2f_dx2 = @(x, y) (4.*x.^5-14.*x.^3+6.*x).*exp(-x.^2-y.^4);
d2f_dxdy = @(x, y) (8.*x.^4-12.*x.^2).*y.^3.*exp(-x.^2-y.^4);
d2f_dy2 = @(x, y) 4.*x.^3*y.^2.*(4.*y.^4-3).*exp(-x.^2-y.^4);
d2f_dydx = @(x, y) 4*x^2*y^3*(2*x^2-3)*exp(-x^2-y^4); 


grad=[df_dx(x(1),y(1)); df_dy(x(1),y(1))];
hess=[d2f_dx2(x(1),y(1)),d2f_dxdy(x(1),y(1));d2f_dydx(x(1),y(1)),d2f_dy2(x(1),y(1))];

if any(eig(hess)<=0)
    fprintf('NON POSITIVE HESSIAN! TERMINATE! \n');
    return;
end

% f(xk+gkdk) MINIMIZATION METHOD
%{
k=1;
while true
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k), y(k)); df_dy(x(k), y(k))];
    hess=[d2f_dx2(x(k),y(k)),d2f_dxdy(x(k),y(k));d2f_dydx(x(k),y(k)),d2f_dy2(x(k),y(k))];
    e = eig(hess)
    
    if any(eig(hess)<=0)
        fprintf('NON POSITIVE HESSIAN! TERMINATE! \n');
        return;
    end
    
    %fprintf('%d \n',norm(grad));
    dk = -hess \ grad;
    %fprintf('[%d, %d ] \n',dk(1),dk(2));
    
    objective_function = @(gamma) f(x(k) + gamma * dk(1), y(k) + gamma * dk(2));
    
    optimal_gamma = gamma_opti(objective_function,0,3);
    %optimal_gamma = fminbnd(objective_function, 0, 5);
    %fprintf('optimal=%d \n',optimal_gamma);
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + optimal_gamma * dk(1);
    y(k+1) = y(k) + optimal_gamma * dk(2);
    
   % Check convergence
    if norm(grad) < e
        break;
    end
    fprintf('norm=%d \n', norm(grad));
    k = k + 1;
end

fprintf('x=%d \n',x(end));
fprintf('y=%d \n',y(end));
%}


% CONSTANT GAMMA MINIMIZATION METHOD
%{
optimal_gamma = 1;
k=1;
while true
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k), y(k)); df_dy(x(k), y(k))];
    hess=[d2f_dx2(x(k),y(k)),d2f_dxdy(x(k),y(k));d2f_dydx(x(k),y(k)),d2f_dy2(x(k),y(k))];
    
    if any(eig(hess)<=0)
        fprintf('NON POSITIVE HESSIAN! TERMINATE! \n');
        return;
    end
    
    %fprintf('%d \n',norm(grad));
    dk = -hess \ grad;
    %fprintf('[%d, %d ] \n',dk(1),dk(2));
    
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + optimal_gamma * dk(1);
    y(k+1) = y(k) + optimal_gamma * dk(2);
    
   % Check convergence
    if norm(grad) < e
        break;
    end
    fprintf('norm=%d \n', norm(grad));
    k = k + 1;
end

fprintf('x=%d \n',x(end));
fprintf('y=%d \n',y(end));
%}


% ARMIJO METHOD
alpha = 1e-5; % You can adjust alpha as needed
beta = 0.5;  % You can adjust beta as needed
s = 1;
gamma=s;
mk = 0;
k=1;
while true
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k), y(k)); df_dy(x(k), y(k))];
    hess=[d2f_dx2(x(k),y(k)),d2f_dxdy(x(k),y(k));d2f_dydx(x(k),y(k)),d2f_dy2(x(k),y(k))];
    e = eig(hess)

    if any(eig(hess)<=0)
        fprintf('NON POSITIVE HESSIAN! TERMINATE! \n');
        return;
    end
    
    %fprintf('%d \n',norm(grad));
    dk = -hess \ grad;
    %fprintf('[%d, %d ] \n',dk(1),dk(2));
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + gamma * dk(1);
    y(k+1) = y(k) + gamma * dk(2);
    
    % Armijo condition
    while f(x(k), y(k))-f(x(k+1), y(k+1)) < -alpha*beta^(mk)*s*dk.'*grad
        mk=mk+1;
    end
    gamma = s*beta^(mk);
    
   % Check convergence
    if norm(grad) < e
        break;
    end
    
    fprintf('norm=%d \n', norm(grad));
    
    k = k + 1;
end

fprintf('x=%d \n',x(end));
fprintf('y=%d \n',y(end));

