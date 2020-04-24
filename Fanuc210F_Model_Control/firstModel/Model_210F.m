%% init
close all
clear all
clc

import +ETS3.* 

%% Robot building
fprintf('Build the robot')

%Define a few positions:
qz=[0 0 0 0 0 0];  %ZERO angle
qr=[0 0 pi/2 0 0 0]; %READY, straight and vertical
qs=[0 -pi/2 pi/2 0 0 0]; %STRECH, straight and horizontal
qn=[0 -pi/4 0 0 -pi/4 0]; %NOMINAL, the arm is in a dextrous working pose
%Matrix of predefined standard positions
POS=[qz; qr; qs; qn];
POS_name=["ZERO";"READY";"STRETCH";"NOMINAL"];

%define DH parameters with the links [METRIC]
%Link      Thet d    a    Alp   r/p offs
L(1)=Link([0    0     0.312 pi/2  0   0   ],'standard');
L(2)=Link([0    0     1.075 0     0   pi/2],'standard'); 
L(3)=Link([0    0     0.225 pi/2  0   0   ],'standard');
L(4)=Link([0    1.280 0     -pi/2  0   0   ],'standard');
L(5)=Link([0    0     0    pi/2  0   0   ],'standard'); 
L(6)=Link([0    0.235 0     0     0   0   ],'standard');

%Set joint limits
Rot_freedom=[360; 136; 362; 720; 250; 720]; %rotational freedom in degrees
Limits=[-Rot_freedom/2 Rot_freedom/2]; % calculate upper and lower limits in degrees
Lim=Limits*pi/180;  %convert to radians
%assign values to joints
L(1).qlim = Lim(1,:);
%Manually adjusted - took maximum rotational freedom and adjusted for
%hoizontal alginment
L(2).qlim = [-pi/2 (-pi/2+Rot_freedom(2)*pi/180)]; %Lim(2,:); 
%Manually adjusted - same freedom in both directions without changing
%offset
L(3).qlim = [Lim(3,1)+pi/2 Lim(3,2)+pi/2];
L(4).qlim = Lim(4,:);
L(5).qlim = Lim(5,:);
L(6).qlim = Lim(6,:);




%create robot object
R= SerialLink(L, 'name', '210F', 'comment', 'six DOF', 'manufacturer', 'FANUC'); %Add base transform if necessary with 'base' and tool transformation with  'tool'

%Put it on a base [METRIC]
R.base = SE3(0, 0, 0.670);
%Turn Robot upside down
%R.base =  SE3(0,0,670) * SE3.Rx(pi);

%Add a tool [METRIC]
toolLength=0.658; %See CAD drawing from Laurence for measurements
R.tool =SE3(0, 0, toolLength); %(0 0  {SE3.rand OR toolLength OR 0})



%Plot the robot
figure(1);
R.plot(qz)
%create teach pendant
R.teach
%output some infos
%r.structure %doesn't work

numoflinks=size(R.links,2);
fprintf('Number of links:\n%d',numoflinks)

fprintf('Is robot of configuration RRRRRR ? (1=Y, 0=N)')
R.isconfig('RRRRRR') %returns 1, if joint config string is right
fprintf('Are all joints of revolute type ? (1=Y, 0=N)')
R.isrevolute %returns 1, for all joints that are revolute joints

R.fast=1; %set to 1 for fast RNE

%%Showall
fprintf('Overview of the robot parameters:\n')
R.display

%Break 
fprintf('press any key to continue...\n')
pause();
%% Forward kinematics
fprintf('forward Kinematics')

for i=1:size(POS,1)
    fprintf('Robot in position %s :\n', POS_name(i,:));
    T(i)=R.fkine(POS(i,:));
    disp(T(i));
    figure(1);
    R.plot(POS(i,:));
    pause(1);
end



%Break 
fprintf('press any key to continue...\n')
pause();
%% Inverse Kinematics

fprintf('inverse kinematics')
%Set robot type
R.ikineType = 'nooffset';

%Configurations: 
cl='l'; %LEFT handed
cr='r'; %Right handed
cu='u'; %elbow UP
cd='d'; %elbow DOWN
cf='f'; %wrist FLIPPED
cn='n'; %wrist NOT flipped
%List of all configs:
% configurations=[[cl,cr],[cu,cd],[cf,cn]];
conf=[
    cl cu cf
    cr cu cf
    cl cd cf
    cr cd cf
    cl cu cn
    cr cu cn
    cl cd cn
    cr cd cn
    ];

fprintf('Inverse kinematics for all defined standard positions in all configurations: \n')
for i = 1 : length(conf)  
    %Sort by configuration iteration
% TI(1,:,i)=R.ikine6s(T(1),conf(i,:))'
% TI(2,:,i)=R.ikine6s(T(2),conf(i,:))'
% TI(3,:,i)=R.ikine6s(T(3),conf(i,:))'
% TI(4,:,i)=R.ikine6s(T(4),conf(i,:))'

%sort by pose
TI(i,:,1)=R.ikine6s(T(1),conf(i,:))'
TI(i,:,2)=R.ikine6s(T(2),conf(i,:))'
TI(i,:,3)=R.ikine6s(T(3),conf(i,:))'
TI(i,:,4)=R.ikine6s(T(4),conf(i,:))'
end

%should deliver same endeffector position, but doesn't!!! OMG
for i= 1:length(TI(:,:,1))
    figure(1);
    R.plot(TI(i,:,1))
    pause(1)
end
fprintf('This gives strange results, as all should point to the same endeffector position... \n \n')


%Same test again for single position, no special matrices, direct feeding
fprintf('Now a small test for a single position with ikine6s: \n')
for i = 1 : length(conf)
TI1(i,:)=R.ikine6s(R.fkine(qr),conf(i,:))'
TF1(i,:)=R.fkine(TI1(i,:))
end
%should be the same as
fprintf('should be the same as: \n')
R.fkine(qr)
%Why no match???

% One of these should be 1!!
fprintf('comparing the previous two results shold deliver at least one match:\n')
for i= 1:size(TI,1)
TI(i,:,1) == T(1)
end

fprintf('Numerical solution:\n')
%Numerical solution works, but is not precise
R.fkine(R.ikine(R.fkine(qr)))
fprintf('should equal:\n')
%equeals
R.fkine(qr)

fprintf('inverse kinematics with ikcon')
%inverse kinematics using optimisation with joint limits - This one works
%perfect
figure(1);
R.plot(qz)
pause(1);
R.plot(R.ikcon(R.fkine(qz)))

fprintf('testing unreachable position:\n')
% Testing unreachable Position
[Q,ERR,EXITFLAG,OUTPUT]=R.ikcon(SE3(30000,0,0))
%doesn't throw proper error message, but at least gives error output
fprintf('does not throw proper error message, but at least gives error output\n')
%List of inverse kinematic functions
% ikine6s 	inverse kinematics for 6-axis spherical wrist revolute robot
% ikine 	inverse kinematics using iterative numerical method
% ikunc 	inverse kinematics using optimisation
% ikcon 	inverse kinematics using optimisation with joint limits
% ikine_sym 	analytic inverse kinematics obtained symbolically

%What *** is wrong with ikine6s? all other solutions work
% R.plot(qn)
% R.plot(R.ikcon(R.fkine(qn)))
% R.plot(R.ikunc(R.fkine(qn)))
% R.plot(R.ikine(R.fkine(qn)))
% R.plot(R.ikine6s(R.fkine(qn)))
% R.plot(R.ikine6s(R.fkine(qn),'luf'))
% R.plot(R.ikine6s(R.fkine(qn),'ruf'))
% R.plot(R.ikine6s(R.fkine(qn),'ldf'))
% R.plot(R.ikine6s(R.fkine(qn),'rdf'))
% R.plot(R.ikine6s(R.fkine(qn),'lun'))
% R.plot(R.ikine6s(R.fkine(qn),'run'))
% R.plot(R.ikine6s(R.fkine(qn),'ldn'))
% R.plot(R.ikine6s(R.fkine(qn),'rdn'))

%Break 
fprintf('press any key to continue...\n')
pause();
%% Trajectories
fprintf('trajectories')

% Joint space motion

%Define two positions in xy plane [METRIC]
T1 = SE3(1.5,  1, 0) * SE3.Rx(pi); 
T2 = SE3(1.5, -1, 0) * SE3.Rx(pi/2);  
%joint coordinate vectors:
q1 = R.ikcon(T1);
q2 = R.ikcon(T2);
%Motion should occur over a time period of 2 sec. in 50ms time steps
t = [0:0.05:2]';
% For mtraj and jtraj the ?nal argument can be a time vector, as here, or an integer 
% specifying the number of time steps.
% q = mtraj(@tpoly, q1, q2, t);
% q = mtraj(@lspb, q1, q2, t);
%For convenience use jtraj
% q = jtraj(q1, q2, t);
%joint velocity and acceleration vectors:
[q,qd,qdd] = jtraj(q1, q2, t); 
%Alternatively, a jatraj method is provided by SerialLink class, but only
%provides position output
% q=R.jtraj(T1,T2,t)
%animate the output:
figure(1);
R.plot(q)
%plot the angles versus time
figure();
qplot(t, q); 
%plot the velocities versus time
figure();
qplot(t, qd); 
%plot the accelerations versus time
figure();
qplot(t, qdd); 
%determine movement of endeffector in cartesian space
clear T
T=R.fkine(q);
p=T.transl;
about(p)
%plot the path in 2D (x,y with z assumed as neglible) and 3D
figure();
plot(p(:,1), p(:,2))
plot3(p(:,1), p(:,2), p(:,3))
%  orientation of the end-effector, in XYZ roll-pitch-yaw angle form
plot(t, T.torpy('xyz'))

%pause();
% Cartesian motion
%where straight line motion in caresian space is reqired. only accepts number of time steps
Ts = ctraj(T1, T2, length(t)); 
%joint space trjaectory
qc = R.ikine(Ts)
%animate the path
figure(1);
R.plot(qc)
%plot the path
figure();
plot(t, Ts.transl);
%Break 
fprintf('press any key to continue...\n')
pause();
 %% Timeseries
% q=[      0         0         0         0         0         0
%          0    0.0365   -0.0365         0         0         0
%          0    0.2273   -0.2273         0         0         0
%          0    0.5779   -0.5779         0         0         0
%          0    0.9929   -0.9929         0         0         0
%          0    1.3435   -1.3435         0         0         0
%          0    1.5343   -1.5343         0         0         0
%          0    1.5708   -1.5708         0         0         0 ]
% 
% T=R.fkine(q)
% 
% about T
% 

% %% Jacobian
% fprintf('Jacobian translational part (upper matrix):\n')
% R.jacobe(qz,'trans')
% fprintf('Jacobian rotational part (lower matrix):\n')
% R.jacobe(qz,'rot')




%% Dynamic Modelling
 
% Volumentric Estimation of Linkweight

%Alternative: STL-file volume estimation. Too big to be implemented now.
%See here for more:
%https://de.mathworks.com/matlabcentral/fileexchange/26982-volume-of-a-surface-triangulation



% Assumed parameter values:
%syms MROB L_H1 L_H2 L_H3 L_H4 L_H5 L_H6 L_R1 L_R2 L_R3 L_R4 L_R5 L_R6 L_D1 L_D2 L_D3 L_D4 L_D5 L_D6 L_W1 L_W2 L_W3 L_W4 L_W5 L_W6 %integer positive
Mtot=   1170 %[Kg] [METRIC] %Total mass of Robot according to datasheet
L_h= [0.280    0.600       0.310     1.280      0.240     0.50     ]%LinkHeight (z)
L_r= [0.600/2  0.600/2     0.310/2   0.180/2    0.235/2   0.150/2  ]%LinkRadius
L_d= [0.600    0.670-0.280 0.310     0.180      0.240     0.150    ]%LinkDepth (y)
L_w= [0.600    0.600       1.075     0.180      0.230     0.150    ]%LinkWidth (x)
L_sh=[1        1           1         0          1         0        ]%Shape (0=cyl 1=cube 1<zeroMass 0>error)

%Volume of links based on cylinder model
for i=1:numoflinks
    h=L_h(i)    %height
    r=L_r(i)    %radius    
    Vcyl(i)=r^2*h*pi
end

%Volume of links based on cuboid model
for i=1:numoflinks
    h=L_h(i)    %height
    d=L_d(i)    %depth
    w=L_w(i)    %width    
    Vcube(i)=h * d * w
end

%Total volume based on shape vector:
Vtot=0
for i=1:numoflinks
    if L_sh(i)==0
        Vtot=Vtot+ Vcyl(i)
    elseif L_sh(i)==1
        Vtot=Vtot+ Vcube(i)
    else
        Vtot=Vtot
    end
end

%Create link mass based on shape vector
for i=1:numoflinks
    if L_sh(i)==0
        Mlink(i)=Mtot*Vcyl(i)/Vtot
    elseif L_sh(i)==1
        Mlink(i)=Mtot*Vcube(i)/Vtot
    else
        Mlink(i)=0
    end
end

% %Mass of links based on cylinder model
% for i=1:numoflinks
%     Mcyl(i)=Mtot*Vcyl(i)/sum(Vcyl)
% end
% 
% %Mass of links based on cuboid model
% for i=1:numoflinks
%     Mcube(i)=Mtot*Vcube(i)/sum(Vcube)
% end

%create innertia tensors based on shape vector
    %Innertia Tensor
for i=1:numoflinks
    if L_sh(i)==0
        m=Mlink(i)   %mass 
        h=L_h(i)    %height
        r=L_r(i)    %depth
        
        Icyl=[
        1/12*m*(3*r^2+h^2)   0                   0  
        0                   1/12*m*(3*r^2+h^2)   0
        0                   0                   1/2*m*r^2
        ]
    
        InTens(:,:,i)= Icyl;    
    elseif L_sh(i)==1
        
        m=Mlink(i)  %mass 
        h=L_h(i)    %height
        d=L_d(i)    %depth
        w=L_w(i)    %width
    
        Icube=[
        1/12*m*(h^2+d^2)    0                   0
        0                   1/12*m*(w^2+d^2)    0
        0                   0                   1/12*m*(w^2+h^2)
        ]
    
        InTens(:,:,i)= Icube;
        
    else
        InTens(:,:,i)=zeros(3)
    end
end


% %calculate Inertia marix based on cylinder model
% for i=1:numoflinks
%     m=Mcyl(i)   %mass 
%     h=L_h(i)    %height
%     r=L_r(i)    %depth
%     Icyl(i,:,:)=[
%         1/12*m*(3*r^2+h^2)   0                   0  
%         0                   1/12*m*(3*r^2+h^2)   0
%         0                   0                   1/2*m*r^2
%         ]
% end
% 
% %Calculate Inertia matrix based on cuboid model
% for i=1:numoflinks
%     m=Mcube(i)  %mass 
%     h=L_h(i)    %height
%     d=L_d(i)    %depth
%     w=L_w(i)    %width
%     Icube(i,:,:)=[
%         1/12*m*(h^2+d^2)    0                   0
%         0                   1/12*m*(w^2+d^2)    0
%         0                   0                   1/12*m*(w^2+h^2)
%         ]
% end

%Estimation of Center of gravity
%R_simple=[L_w'/2 L_d'/2 L_h'/2]; %WRONG! only offset along axis
R_simple=[zeros(size(L_w'/2)) zeros(size(L_d'/2)) L_h'/2];
%Correct for DH-Frame/ physical link deviation
R_corrected=[ %Corrective Matrix for link base displacement
    0   0   0
    0   0   0
    0   0   0
    0   0   0
    0   0   0
    0   0   0
] + R_simple;

%Steal fricion model form other Robot.
%Here: puma560
mdl_puma560
for i=1:numoflinks %has to be grabbed 1 by 1
Lfr_stolen(i,:)=p560.links(:,i).Tc; %Linkfriction %fr_stolen because very vague friction
MB_stolen(i,:)=p560.links(:,i).B;    %Motor viscous friction
gears_stolen(i,:)=p560.links(:,i).G; %Gear ratio
MI_stolen(i,:) =p560.links(:,i).Jm;  %Motor innertia
end 
    

%Assign parameters to Links
for i=1:numoflinks
    R.links(i).m=Mlink(i);
    R.links(i).I=InTens(:,:,i);
    R.links(i).r=R_corrected(i,:);
    R.links(i).Tc=Lfr_stolen(i,:);
    R.links(i).B = MB_stolen(i,:); 
    R.links(i).G = gears_stolen(i,:);
    R.links(i).Jm = MI_stolen(i,:);     
end

%Display dynamics:
R.dyn

%Set weight of the endeffector with payload command (Endeffector is assumed
%massless, so combined weight of endeffector and payload is defined here
%at their commmon center of mass
toolmass=25 % Value given by Laurence
R.payload(toolmass, [0 0 0.5*toolLength ]); %point mass of 25 kg is assumed at half the lenght of tool 0.5*toolLength


%Forces for holding the robot in normal position:
Q = R.rne(POS(4,:), POS(1,:), POS(1,:)) %Nonzero due to gravity
%Forces necessary to hold robot arm at different steps between two positions (See moving gravity vector) 
q = jtraj(POS(1,:), POS(4,:), 10)
Q = R.rne(q, 0*q, 0*q)
%Zero gravity
R.rne(POS(4,:), POS(1,:), POS(1,:), 'gravity', [0 0 0]) %should give zero forces
%Demonstrate influence of innertia on other links
R.rne(POS(4,:), [1 0 0 0 0 0], POS(1,:), 'gravity', [0 0 0]) 
%Demonstrate Gravity loads for different poses:
for i=1:size(POS,1)
    R.gravload(POS(i,:))
end

%calculate the forces needed for different positions 
[Q2,Q3] = meshgrid(-pi:0.1:pi, -pi:0.1:pi); 
for i=1:numcols(Q2),
for j=1:numcols(Q3);
g = R.gravload([0 Q2(i,j) Q3(i,j) 0 0 0]);
g2(i,j) = g(2);
g3(i,j) = g(3);
%Simulate them all!
%R.plot([0 Q2(i,j) Q3(i,j) 0 0 0])
end
end
figure()
surfl(Q2, Q3, g2); 
figure()
surfl(Q2, Q3, g3);

%
figure(1)
%[t q qd] = R.nofriction().fdyn(10, [], POS(4,:));
[t q qd] = R.nofriction().fdyn(2, [], POS(1,:));
figure(1)
R.plot(q)



%% Jacobian

% Section of a of a hyperellipsoid in Cartesian acceleration space. 
% The major axis of this ellipsoid is the direction in which 
% the manipulator has maximum acceleration at this con?guration.

% pose = qs;
% J = R.jacob0(pose);
% M = R.inertia(pose);
% Mx = (J * inv(M) * inv(M)' * J');
% Mx = Mx(1:3, 1:3);
% figure()
% plot_ellipse( Mx )

%% clean up the graphs
%Break 
fprintf('press any key to continue and close all figures? strg-c aborts and leaves all figures opened...\n')
pause();
close figure 2
close figure 3
close figure 4
close figure 5
close figure 6
close figure 7
close figure 8
%close figure 9
%close figure 10