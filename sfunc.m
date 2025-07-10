function y=sfunc(x,type,params)
switch type
    case 'exp'
        y = 1-exp(-params(1)*x);
    case 'sigma'
        y = 1 ./ (1 + exp(-params(1) * (x - params(2))));
    case 'sigman'
        y = 1 ./ (1 + exp(-10*params(1)/params(2) * (x - params(2)/2)));
    case 'randomized'
        y = abs(mvnrnd(x, params(1) + x * 0));
        y = y / sum(y);
        if numel(y)>1
            y = y / sum(y);
        end
    case 'randomizedSigma'
        y = 1 ./ (1 + exp(-params(1) * (x - params(2))));
        y = abs(mvnrnd(y, params(3) + y * 0));

        if numel(y)>1
            y = y / sum(y);
        end

end
end
