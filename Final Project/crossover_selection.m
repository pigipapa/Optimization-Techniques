function crossover_population = crossover_selection(best_population, crossover_popul_num, crossover_method)

    if crossover_method == "Cut"
        
        offspring_num = 0;
        
        crossover_population = [];
        
        while offspring_num < crossover_popul_num
        
        
            crossover_point = randi(size(best_population, 2));
            
            parents_random = randperm(size(best_population, 1), 2);
            parent1 = best_population(parents_random(1), :);
            parent2 = best_population(parents_random(2), :);
            
            offspring1 = [parent1(1:crossover_point), parent2(crossover_point+1:end)];
            offspring2 = [parent2(1:crossover_point), parent1(crossover_point+1:end)];
            crossover_population = [crossover_population; offspring1; offspring2];
        
            offspring_num = offspring_num + 2;
        end 
        
    elseif crossover_method == "Merge"
        
        offspring_num = 0;
        
        crossover_population = [];
        
        while offspring_num < crossover_popul_num
        
        
            crossover_point = randi(size(best_population, 2));
            
            parents_random = randperm(size(best_population, 1), 2);
            parent1 = best_population(parents_random(1), :);
            parent2 = best_population(parents_random(2), :);
            
            offspring = [parent2(1:crossover_point), parent1(crossover_point+1:end)];
            crossover_population = [crossover_population; offspring];
        
            offspring_num = offspring_num + 1;
        end 
    
    end


end