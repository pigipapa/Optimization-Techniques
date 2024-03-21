function new_population = random_selection(population, random_popul_num)

    random_indices = randperm(size(population, 1), random_popul_num);

    selected_rows = population(random_indices(1:random_popul_num), :);

    new_population = selected_rows;

end