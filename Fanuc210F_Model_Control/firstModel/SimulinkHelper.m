

%% Puma
% 
% tic
% sim('sl_opspace')
% a=toc
tic
scatter3(opspace.position.Data(:,1),opspace.position.Data(:,2),opspace.position.Data(:,3),[],abs(sum(opspace.force.Data,2)))
b=toc
tic
p560.plot(opspace.q_live.Data)
c=toc

d=a+b+c
save('timings.mat')


%% Fanuc
 
% tic
% sim('SimulationAndControl_Fanuc210F')
% a=toc
tic
scatter3(out.position.Data(:,1),out.position.Data(:,2),out.position.Data(:,3),[],abs(sum(out.force.Data,2)))
b=toc
tic
R.plot(out.q_live.Data)
c=toc
d=a+b+c

%% Remove the evidence for a new plot
clear out.position
clear out.q_live