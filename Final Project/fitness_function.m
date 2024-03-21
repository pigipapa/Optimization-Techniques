function [error_result] = fitness_function(genes, num_of_gaussian, f, error_type, u1_lim, u2_lim)

    chromosomeSize = length(genes)/num_of_gaussian;
    
    error_result = 0;
    spacing = 20;
    
    for u1=linspace(u1_lim(1),u1_lim(2),spacing)
        for u2=linspace(u2_lim(1),u2_lim(2),spacing)
            
            f_value = f(u1,u2);
            f_bar = 0;

            for j=1:chromosomeSize:length(genes)
                gaussian_value = gaussian_function(u1, u2, genes(j), genes(j+1), genes(j+2), genes(j+3), genes(j+4));
                f_bar = f_bar + gaussian_value;
            end

            if error_type == "Linear"

                error_result = error_result + abs(f_value - f_bar);

            elseif error_type == "Mean Square"

                error_result = error_result + (f_value - f_bar)^2;

            end
        end
    end
    
    error_result = error_result/(spacing^2);



end
