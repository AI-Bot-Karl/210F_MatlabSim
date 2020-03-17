clear all
close all
mdl_ball
t = [0:0.05:0.5]';
q_=q;
ball.tool =SE3(0, 0, 1);
while true
qnew =rand(size(q));
ball.plot(jtraj(q_, qnew, t));
q_= qnew;
end