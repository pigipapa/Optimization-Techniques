function g = gamma_opti(fun, ak, bk)

    e=0.001;
    l=0.005;    

    k=1;
    % repeat until ak and bk are closer than l
    while (bk-ak>=l)
        
        % given algorithm
        x1k=(ak+bk)/2-e;
        x2k=(ak+bk)/2+e;
        if fun(x1k)<fun(x2k)
        ak=ak; 
        bk=x2k;
        else
        ak=x1k; 
        bk=bk; 
        end   
        k=k+1;
    end
    
    g=ak;

end