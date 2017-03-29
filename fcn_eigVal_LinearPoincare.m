function eigVal = fcn_eigVal_LinearPoincare(k,t)

eigVal(1) = (exp(-k*t)*(exp(k*t) + (exp(2*k*t) - 2*exp(k*t) + 4*exp(2*k*t)*exp(t) + 1)^(1/2) + 1))/2;
eigVal(2) = (exp(-k*t)*(exp(k*t) - (exp(2*k*t) - 2*exp(k*t) + 4*exp(2*k*t)*exp(t) + 1)^(1/2) + 1))/2;