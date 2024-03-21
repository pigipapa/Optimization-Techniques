 function [f_approx] = plot_approx_f(resolution, final_gene, chromosome_size, u1_lim, u2_lim, generation)

    f_approx = zeros(resolution);

    y_counter = 0;

    for u1=linspace(-1,2,resolution)
        y_counter = y_counter + 1;
        x_counter = 0;
        for u2=linspace(-2,1,resolution)
            x_counter = x_counter + 1;
        for j=1:chromosome_size:length(final_gene)
                    gaussian_value = gaussian_function(u1, u2, final_gene(j), final_gene(j+1), final_gene(j+2), final_gene(j+3), final_gene(j+4));
                    f_approx(x_counter, y_counter)  = f_approx(x_counter, y_counter) + gaussian_value;
        end
        end
    end

    num_of_gaussians = length(final_gene)/chromosome_size;
    
    u1 = linspace(u1_lim(1),u1_lim(2),resolution);
    u2 = linspace(u2_lim(1),u2_lim(2),resolution);
    figure();
    surf(u1,u2, f_approx);

    xlabel('u1');
    ylabel('u2');
    zlabel('f');
    title({'Analytic function'
            ['Number of gaussians = ' num2str(num_of_gaussians)]
            ['Number of generations = ' num2str(generation)] });
    grid on;

end