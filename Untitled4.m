syms k t
A = [0 1;0 -k];
exp(A)
Phi = [1 exp(t);1 exp(-k*t)];

% eig(Phi)