function [selected_indices,values] = biasedRandomSelection(vec, s, biasFactor, randomize_probability)
    % biasedRandomSelection selects 's' elements from 'vec' with a bias toward higher values
    % vec - input vector to select from
    % s - number of elements to select
    % biasFactor - controls the strength of the bias
    % randomize_probability - the probability of selecting randomly
    vec=vec(:);
    N = length(vec);
    
    if N == 0 || s <= 0
        selected_indices = [];
        return;
    end
    
    % Sort 'vec' in descending order and store the sorted indices
    [values, index] = sort(vec, 'descend');
    
    % Handle case where 's' exceeds the length of 'vec'
    if N < s
        selected_indices = [ones(1, s - N), 1:N];
        return;
    end
    
    selected_indices = zeros(1, s);
    
    % Create a probability distribution biased toward higher values (beginning)
    weights = exp(-biasFactor * (1:N) / N);  % Exponentially decaying weights
    cumulative_weights = cumsum(weights / sum(weights));  % Normalize to cumulative weights
    
    % Vectorized selection for biased elements
    rand_vals = rand(1, s);  % Generate all random values at once
    biased_selections = discretize(rand_vals, [0 cumulative_weights]);
    
    % Randomize some selections based on the randomize_probability
    random_selection_flags = rand(1, s) > randomize_probability;
    random_indices = randi(N, 1, sum(random_selection_flags));
    
    % Apply the bias and random selection
    selected_indices(~random_selection_flags) = index(biased_selections(~random_selection_flags));
    selected_indices(random_selection_flags) = index(random_indices);

    values=values(selected_indices);
end
