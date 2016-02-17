%% load in
fprintf('Setup...\n');
dt = 0.001;
options.dt = dt;
options.floating = false;
options.base_offset = [0, 0, 0]';
options.base_rpy = [0, 0, 0]';
options.ignore_self_collisions = true;
options.collision = false;
options.hands = 'none';
r = LWR('urdf/lwr.urdf', options);

v=r.constructVisualizer();

%% initial config
x0 = r.getInitialState();

%% final pose
fprintf('Generating target traj\n');
options.visualize = true;
target_xtraj = runPlanning(x0(1:r.getNumPositions), [0.5, 0.0, 0.5]', options);

% TODO: no control yet
pd_control = irb140_trajfollow_block(r, target_xtraj);
clear ins; clear connection_1to2;
clear outs; clear connection_2to1;
ins(1).system = 1;
ins(1).input = 2;
outs(1).system = 1;
outs(1).output = 1;
outs(2).system = 1;
outs(2).output = 2;
connection_1to2(1).from_output = 1;
connection_1to2(1).to_input = 1;
connection_2to1(1).from_output = 1;
connection_2to1(1).to_input = 1;
sys = mimoFeedback(r, pd_control, connection_1to2, connection_2to1, ins,outs);

%% simulate
v=r.constructVisualizer();
v.display_dt = 0.001;
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
output_select(2).system=1;
output_select(2).output=2;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

traj = simulate(sys,[0 2],x0);

playback(v,traj,struct('slider',true));