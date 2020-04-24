syms q1 q2 q3 q4 q5 q6 q1d q2d q3d q4d q5d q6d  q1dd q2dd q3dd q4dd q5dd q6dd real
% tic
% tau_p560 = p560.rne([q1 q2 q3 q4 q5 q6], [q1d q2d q3d q4d q5d q6d], [q1dd q2dd q3dd q4dd q5dd q6dd])
% toc
% save
tic
R.fast=0
which rne
tau_R = R.rne([q1 q2 q3 q4 q5 q6], [q1d q2d q3d q4d q5d q6d], [q1dd q2dd q3dd q4dd q5dd q6dd])
R.fast=1
toc
save

 R.inertia([q1 q2 q3 q4 q5 q6])