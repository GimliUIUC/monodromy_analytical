% Monodromy Matrix Derivation
% Author: Yanran Ding

M = 0.675;
len_k = 10;
len_Tst = 10;
k_span = linspace(0,4,len_k);
Tst_span = linspace(0.05,0.5,len_Tst);
[k, Tst] = meshgrid(k_span,Tst_span);
eig_max = zeros(len_k,len_Tst);


for ii = 1:len_k
    for jj = 1:len_Tst
        % jump dynamics: dq = f1 = [dz;ddz] = [dz;Fz/M - g + k/M*(dz_d-dz)]
        % df1_dq = [0 1;0 -k/M] = dQ1_dt
        % Q1 = expm([0 1;0 -k/M]*Tst) <-- Monodromy at takeoff
        % flight dynamics: dq = f2 = [dz;ddz] = [dz;-g]
        % df2_dq = [0 1;0 0] = dQ2_dt
        % Q2 = expm([0 1;0 0]*Tfl) <-- Monodromy at the apex from takeoff
        % Q = Q2*Q1
%         Q = expm([0 1;0 0])*expm([0 1;0 -k(i)/M]);
        Qapex = expm([0 1;0 0])*expm([0 1;0 -k(ii)/M]*Tst(jj));
        [eigVec, eigVal] = eig(Qapex);
        fprintf('eigenvales are: %.2f, %.2f \n',eigVal(1,1),eigVal(2,2))
        fprintf('eigenvectors are: [%.2f;%.2f],[%.2f,%.2f] \n',eigVec(:,1),eigVec(:,2))
    end
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
