function h=plotBarXY(r,mean,std,orientation,symbol,varargin)
hold all
if orientation=='h'
    h(1)=semilogy(mean, r,symbol, varargin{:});
    h(2)=semilogy([mean-std,mean+std], [r, r], varargin{:});
else
    h(1)=semilogy([r, r], [mean-std,mean+std], varargin{:});
    h(2)=semilogy(r,mean, symbol, varargin{:});
end

