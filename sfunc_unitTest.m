figure

L=1;
x=linspace(-1e-3,0.5,100);

y=sfunc(x,'sigma',[10 0])
plot(x,y)