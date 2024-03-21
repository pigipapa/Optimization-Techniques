function mutated_chromosome = mutation(chromosome_to_mutate, num_of_genes, c_lim, sigma_lim, d_lim)
   
    N = length(chromosome_to_mutate);
    
    mutated_index = randi(N);
    
    mutated_chromosome = chromosome_to_mutate;

    while mutated_chromosome(mutated_index) == chromosome_to_mutate(mutated_index)
        
        switch mod(mutated_index,num_of_genes)
            case 1
                mutated_chromosome(mutated_index) = unifrnd(c_lim(1), c_lim(2));
            case 2
                mutated_chromosome(mutated_index) = unifrnd(c_lim(1), c_lim(2));
            case 3
                mutated_chromosome(mutated_index) = unifrnd(sigma_lim(1), sigma_lim(2));
            case 4
                mutated_chromosome(mutated_index) = unifrnd(sigma_lim(1), sigma_lim(2));
            case 0
                mutated_chromosome(mutated_index) = unifrnd(d_lim(1), d_lim(2));
        end
        
    end


end