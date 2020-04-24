%% Fanuc
 
% tic
% sim('SimulationAndControl_Fanuc210F_2')
% a=toc
tic
scatter3(Rsim.X.Data,Rsim.Y.Data,Rsim.Z.Data, abs(normalize(sum((Rsim.Q.Data),2)))) %abs(sum(Rsim.Q.Data,2))
b=toc
tic
R.plot(Rsim.q.Data, 'fps', 900)
c=toc
d=a+b+c
