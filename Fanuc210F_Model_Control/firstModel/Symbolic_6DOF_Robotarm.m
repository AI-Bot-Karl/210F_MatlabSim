
%Define symbolic kinematics

syms a_1 a_2 a_3 a_4 a_5 a_6
syms d_1 d_2 d_3 d_4 d_5 d_6
syms alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6

%Link      Thet d    a    Alp   r/p offs
L_sym(1)=Link([0    0     a_1  alpha_1  0   0   ],'standard');
L_sym(2)=Link([0    0     a_2 0     0    pi/2],'standard'); %Maybe [0    0    1075 0     0   -pi/2]; NOW:depending on direction determied with graphical representation
L_sym(3)=Link([0    0     a_3  alpha_3  0   0   ],'standard');
L_sym(4)=Link([0    d_4 0      alpha_4  0   0   ],'standard');
L_sym(5)=Link([0    0     0     alpha_5  0   0   ],'standard'); %Maybe [0    0    0    pi/2 0   0   ]; NOW: depending on direction determied with graphical representation
L_sym(6)=Link([0    d_6 0     0     0   0   ],'standard');


sixlink= SerialLink(L_sym, 'name', 'twolink', 'comment', 'six DOF', 'manufacturer', 'generic'); %Add base transform if necessary with 'base' and tool transformation with  'tool'

%Assign dynamic parameters to Links

for i=1:numoflinks
    R.links(i).m=Mlink(i);
    R.links(i).I=InTens(:,:,i);
    R.links(i).r=R_corrected(i,:);
    R.links(i).Tc=Lfr_stolen(i,:)
    R.links(i).B = MB_stolen(i,:) 
    R.links(i).G = gears_stolen(i,:)
    R.links(i).Jm = MI_stolen(i,:)     
end