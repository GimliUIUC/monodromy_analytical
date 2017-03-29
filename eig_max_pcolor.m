% Monodromy Matrix Derivation
% Author: Yanran Ding

k = 50;
% k = linspace(0,4,10);
% T_st = linspace(0.05,0.2,10);
eig_max = zeros(size(k));
M = 0.675;
Tst = 0.18;             % stance time
Tflt = 0.6;             % flight time

%% 
% jump dynamics: dq = f1 = [dz;ddz] = [dz;Fz/M - g + k/M*(dz_d-dz)]
% df1_dq = [0 1;0 -k/M] = dQ1_dt
% flight dynamics: dq = f2 = [dz;ddz] = [dz;-g]
% df2_dq = [0 1;0 0] = dQ2_dt
% Q = expm(dQ1_dt*Tst)*expm(dQ2_dt*Tflt) <-- The monodromy matrix

for i = 1:length(k)
    Q = expm([0 1;0 0]*Tflt)*expm([0 1;0 -k/M]*Tst);
    Q = Q
    [eigVec, eigVal] = eig(Q)
%     fprintf('eigenvales are: %.2f, %.2f \n',eigVal(1,1),eigVal(2,2))
%     fprintf('eigenvectors are: [%.2f;%.2f],[%.2f,%.2f] \n',eigVec(:,1),eigVec(:,2))
end

% plot(k,eig_max)
% xlabel('k, feedback gain')
% ylabel('maximum eigen-value for Monodromy Matrix')
% hold on;
% plot(k,eig_min,'r')


% pcolor(k,t,eig_max)
% shading flat
% hold on
% [C,H]=contour(k,t,eig_max,'-k','LevelStep',0.1);
% clabel(C,H);
% xlabel('k')
% ylabel('T')
% title('eig max')
% colorbar
% caxis([0 0.2])
% pbaspect([2 1 1])
