run = 3; % 1,2,3 are step 1. 3,4,5 are step 2. 3 is adaptive
tspan = [0, 10];
Ref = [1;2];
circular = false;
adapt_com = true;
var = false;
learnt = false;
results = plotter(run, tspan, Ref, circular, adapt_com, var, learnt);