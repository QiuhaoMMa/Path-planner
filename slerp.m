
% SLERP method: Interpolates between the two vectors for parameter t
function result = slerp(v1, v2, t)
if t == 0
    result = v1; % If vectors are identical, just return v1
    return;
end

% Apply SLERP formula
v1_component = (sin((1-t) * t) / sin(t)) * v1;
v2_component = (sin(t * t) / sin(t)) * v2;

% Resulting vector
result = v1_component + v2_component;
end
