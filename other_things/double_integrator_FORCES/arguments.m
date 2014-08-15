
o{1} = [1 4; 1 2; 5 2; 5 4];
o{2} = [1.5 9.5;1.5 6.5; 2.5 6.5; 2.5 9.5];
o{3} = [6 8; 6 6; 8 6; 8 8];
args.N = 12;args.z0 = x;
args.nX = 4;args.nU=2;args.nO=length(o); % num of obstacles changes
args.mu = penalty_coeff;
args.x_min = -10;args.x_max = 10;
args.v_min = -1;args.v_max = 1;
args.u_min = -1;args.u_max = 1;
args.x_start=x(1:4);
args.x_goal = x((args.N-1)*args.nX+1:(args.N-1)*args.nX+args.nX);
args.trust_box_size=trust_box_size;
args.obstacles = o;
args.d_safe = 0.05;

