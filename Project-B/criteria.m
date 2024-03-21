function flag = criteria(f, df_dx, df_dy, grad, x, y, gk, dk)

    criterion3 = false;
    criterion4 = false;

    xk = x(end-1);
    yk = y(end-1);

    %Update x and y based on the optimal gamma
    xk_1 = x(end);
    yk_1 = y(end);
    
    grad_f = grad;
    grad_f_1 = [df_dx(xk_1, yk_1); df_dy(xk_1, yk_1)]; 

     for beta = linspace(0.001,1,50)
        if dk'*grad_f_1 <= beta*dk'*grad_f
            continue;
        else
            criterion3 = true;
            break;
        end
    end
        
    if criterion3
        for alpha = linspace(0.001,beta,50)
            if f(xk_1, yk_1) > f(xk, yk) + alpha*gk*dk'*grad_f
                continue
            else
                criterion4 = true;
                break;
            end
        end
    end
    
    flag = criterion3 && criterion4;

end