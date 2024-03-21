clear all;

% Parameters

num_of_chromosomes = 5;

num_of_gaussian = 15;
population_size = 100;
max_generations = 10000; 


best_percentage = 0.2;
random_percentage = 0.1;
crossover_percentage = 1 - (best_percentage + random_percentage);


u1_lim = [-1 2];
u2_lim = [-2 1];
c_lim = [-2 2];
sigma_lim = [0.1 2];

f = @(u1,u2) sin(u1 + u2)*sin(u2^2);

[f_min, f_max] = find_bounds(f, u1_lim, u2_lim);
d_lim = [f_min f_max]; 

min_error = 0.001;
    
crossover_method = ["Cut" "Merge"];
error_type = ["Linear" "Mean Square"];  

% set genes for the first generation

generation = 1;
genes = zeros(population_size, num_of_chromosomes*num_of_gaussian);
fitness_error_results = zeros(population_size, 1);

for i=1:population_size
    
    % uniform distribution
    for j=1:num_of_chromosomes:num_of_gaussian*num_of_chromosomes
        genes(i,j) = unifrnd(c_lim(1), c_lim(2));                       % c1
        genes(i,j+1) =  unifrnd(c_lim(1), c_lim(2));                    % c2
        genes(i,j+2) =  unifrnd(sigma_lim(1), sigma_lim(2));            % sigma1
        genes(i,j+3) =  unifrnd(sigma_lim(1), sigma_lim(2));            % sigma2
        genes(i,j+4) = unifrnd(d_lim(1), d_lim(2));                     % d
    end
    
    fitness_error_results(i) = fitness_function(genes(i,:), num_of_gaussian, f, error_type(2), u1_lim, u2_lim);

end

population = [genes fitness_error_results];

population = sortrows(population, size(population,2));

best_gene(generation,:) = population(1,1:end-1);
minimum_error(generation) = population(1,end);

%generation of new chromosomes

mutation_propability = 0.1;

best_popul_num = population_size*best_percentage;
random_popul_num = population_size*random_percentage;
crossover_popul_num = int8(population_size*crossover_percentage);

random_start = best_popul_num +1;
random_end = random_start + random_popul_num - 1;

while minimum_error(generation) > min_error && generation < max_generations
    
    fprintf("minimum_error = %f \n", minimum_error(generation))
    
    
    generation = generation + 1;
    
    best_population = population(1:best_popul_num, 1:end-1);
    
    population(random_start:random_end, 1:end-1) = random_selection(population(random_start:end, 1:end-1), random_popul_num);
    
    population(random_end+1:end, 1:end-1) = crossover_selection(best_population, crossover_popul_num, crossover_method(1));
    
    
    
    
    for i = 1:population_size
        if rand <= mutation_propability
            mutated_gene_index = randi(population_size);
            mutated_gene = population(mutated_gene_index, 1:end-1);
            population(mutated_gene_index,1:end-1) = mutation(mutated_gene, num_of_chromosomes, c_lim, sigma_lim, d_lim);
        end
    end
    
    
    
    
    for i = 1:population_size
        fitness_error_results(i) = fitness_function(population(i,1:end-1), num_of_gaussian, f, error_type(2), u1_lim, u2_lim);
    end
   
    population(:,end) = fitness_error_results;
    
    population = sortrows(population, size(population,2));

    best_gene(generation,:) = population(1,1:end-1);
    minimum_error(generation) = population(1,end);
    
    
    
end



figure();

hold on;
num_gen = 1:1:generation;
plot(num_gen,minimum_error(num_gen), 'DisplayName',['Number of Gaussians  = ' num2str(num_of_gaussian )]);
xlabel('Number of Generations');
ylabel('Fitness Function Value');
title('Fitness Function Results per Generation');
legend();
grid on;


[finalError, best_generation] = min(minimum_error);
final_gene = best_gene(best_generation, :);

resolution = 30;

f_approx = plot_approx_f(resolution, final_gene, num_of_chromosomes, u1_lim, u2_lim, generation);
