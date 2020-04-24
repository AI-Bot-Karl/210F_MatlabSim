fclear all
close all
mdl_ball
t = [0:0.05:0.5]';
q_=q;
QD0=zeros(size(q));
ball.tool =SE3(0, 0, 1);
i=0;
tic
while true
    i=i+1;
qnew =rand(size(q));
QDF=rand(size(q));
[Q,QD,QDD]=jtraj(q_, qnew, t, QD0, QDF);
ball.plot(Q);
q_= qnew;
QD0=QDF;
%Introduce Memory Leak
positionMemory(i,:)=qnew;
velocityMemory(i,:)=QDF;
disp(size(positionMemory))
TimeMemory(i)=toc;
disp(TimeMemory(i))
end

%Can still be done afterwards
mean(diff(TimeMemory))
min(diff(TimeMemory))
max(diff(TimeMemory))