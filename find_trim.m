% [ state_trim, control_trim ] = find_trim( velocity, altitude, xcg )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                  find_trim.m                           %%
%%  This is the function to find the trim condition for   %%
%%  straight&level flight at given velocity and altitude. %%
%%  The trim data are derived by optimizing the cost      %%
%%  function cost_energy.m using simplex search.          %%
%% ---- Input Variables --------                          %%
%% velocity (ft/sec)- true velocity                       %%
%% altitude (ft)    - altitude                            %%
%% xcg              - center of gravity position as       %%
%%                    fraction of mean aerodynamic chord  %% 
%%                                                        %%
%% ---- Output Variables --------                         %%
%% state_trim   - trim data for the state vector where    %%
%%                                                        %%
%%               state vector =                          %%
%%                                                        %%
%%               [ 1.velocity             ( ft/sec )       %%  
%%                 2. angle of attack      ( rad )          %%
%%                 3. sideslip angle       ( rad )          %%
%%                 4. phi   - Euler angle  ( rad )          %%
%%%                5. theta - Euler angle  ( rad )          %%
%%%                6. psi   - Euler angle  ( rad )          %%
%%                 7. roll rate            ( rad/sec )      %%
%%                 8. pitch rate           ( rad/sec )      %%
%%                 9. yaw rate             ( rad/sec )      %%
%%                 10.north displacement   ( ft )           %%
%%                 11.east displacement    ( ft )           %%    
%%                 12.altitude             ( ft )           %%
%%                 13.power  ( percent, 0 <= pow <= 100 ) ] %%
%%                                                        %%
%% control_trim - trim data for the control vector where  %%
%%                                                        %%
%%                control vector =                        %%
%%                                                        %%
%%                [ throttle setting ( 0 - 1 )            %%
%%                  elevon           ( deg )              %%
%%                  aileron          ( deg )              %% 
%%                  rudder           ( deg ) ];           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ state_trim, control_trim ] = find_trim( velocity, altitude, xcg )
%---- flight steady_states --------------------------
global climb_angle; % rate-of-climbing steady_state
% climb_angle = input( 'Input the climb angel(deg)' );
climb_angle = 0.0;
global coordi_turn; % coordinate turn steady_state
coordi_turn = 0;
global stab; % stability-axis roll steady_state  
stab = 0;
global skid_turn; % skidding turn steady_state
skid_turn = 0;
global rad_gamma; % flight path angle gamma in radian
rad_gamma = 0;
global phi_r; % reference phi
phi_r = 0;
global roll_rate; % reference roll rate
roll_rate = 0;
global pitch_rate; % reference pitch rate
pitch_rate = 0;

% OUTPUTS: trimmed values for states and controls
% INPUTS:  guess values for thrust, elevator, alpha  (assuming steady level flight)
% Initial Guess for free parameters
phi_weight = 10; theta_weight = 10; psi_weight = 10;
phi = 0; psi = 0;
p = 0; q = 0; r = 0;
disp('At what flight condition would you like to trim the F-16?');
disp('1.  Steady Wings-Level Flight.');
disp('2.  Steady Turning Flight.');
disp('3.  Steady Pull-Up Flight.');
disp('4.  Steady Roll Flight.');
FC_flag = input('Your Selection:  ');
switch FC_flag
    case 1
        disp('Trimming for Steady Wings-Level Flight.');
        % do nothing
    case 2
        disp('Trimming for Steady Turning Flight.');
        r = input('Enter the turning rate (deg/s):  ');
        psi_weight = 0;
    case 3
        disp('Trimming for Steady Pull-Up Flight.');
        q = input('Enter the pull-up rate (deg/s):  ');
        theta_weight = 0;
    case 4    
        disp('Trimming for Steady Roll Flight.');
        p = input('Enter the Roll rate    (deg/s):  ');
        phi_weight = 0;
    otherwise
        disp('Invalid Selection')
%        break;
end

weight = [ 1     100       100        phi_weight    theta_weight  psi_weight   10     10    10    0    0     0  0 ];
%weight = [ V_dot alpha_dpt  beta_dot  phi_dot       theta_dot     psi_dot      P_dot  Q_dot R_dot P_do Q_dot h  power];
%---- data ---------------------------
rtod = 57.29577951; % radian to degree
no_step = 5000; % no. of iteration steps for trimming
disp('  ');
read_no = input('Please input the # of trim iterations ( default = 5000 ):  ');
if read_no == []
    no_step = read_no;
end
disp('  ');
disp('----------------------------------------------------');
epsilon = -1.0;
%---- initial condition ----
x0 = [ velocity; 0.0; 0.0; 0.0; 0.0; 0.0; p  ; q  ; r  ; 0.0; 0.0; altitude; 90 ];
    %[  1      ; 2  ; 3  ; 4  ; 5 ;  6  ; 7  ; 8  ; 9  ; 10 ; 11 ; 12      ; 13 ]   
 u0 = [  0.73; -1.0; 0.0 ; 0.0]; 
    % [  T   ;  E  ; A   ; R  ];
%---- define initial steady state for iteration --------------------
s = [ u0(1); u0(2); x0(2); u0(3); u0(4); x0(3) ];
ds = [ 0.2; 1.0; 0.02; 1.0; 1.0; 0.02 ];

%---- simplex(trim) algorithm -----------------
init_cost =  cost_energy( x0, u0, s, xcg,weight);
[ s_trim, f_final ] = ss_trim( s, ds, x0, u0, no_step, epsilon, xcg, weight );

%---- output the trim result ------------
control_trim = u0;
control_trim(1) = s_trim(1);
control_trim(2) = s_trim(2);
control_trim(3) = s_trim(4);
control_trim(4) = s_trim(5);
state = x0;
state(2) = s_trim(3);
state(3) = s_trim(6);
final_cost = cost_energy( state, control_trim, s_trim, xcg,weight );
state(13) = p_thro( control_trim(1) );
state_trim = steady_state( state );
control_trim;

