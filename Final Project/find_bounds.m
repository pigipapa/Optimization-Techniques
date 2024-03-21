function [min_f, max_f] = find_bounds(f, u1_lim, u2_lim)

    step = 0.05;
    
    min_f = Inf;
    max_f = -Inf;
    
    for u1 = u1_lim(1):step:u1_lim(2)

        for u2 = u2_lim(1):step:u2_lim(2)

            fVal = f(u1, u2);

            if fVal < min_f
                min_f = fVal;
            
            elseif fVal > max_f
                max_f = fVal;
                
            end
           
        end
        
    end
    
end