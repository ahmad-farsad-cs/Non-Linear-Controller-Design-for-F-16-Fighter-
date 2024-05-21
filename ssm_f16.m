%{ 
    SSM_F16.m
    x_dot = A * x + B * u
       y  = C * x + D * u
    [ A, B, C, D ] = SSM_F16( velocity, altitude, xcg ) 
    A, B, C, D are state space matrix of linearized model
    for variouus flight conditions 
    i.e. 1. traight 
         2. level flight at  sepecified velocity
         3. altitude 
         4. c.g. position with zero banking angle
also  longitudinal and the lateral states are decoupled.   
        
system Control ( input)   
    u=[ thro; el; ail ]
where 
    thro ( 0 ~ 1.0 )     throttle setting 
    el   ( deg )         elevon deflection
    ail  ( deg )         aileron deflection     
                                                                           
sysetm Stat
   x = [ vt; h; alpha; theta; Q; pow; beta; phi; P;R]
where 
     vt    ( ft/sec )     velocity                 
     h     ( ft )         altitude                 
     alpha ( rad )        angle of attack          
     theta ( rad )        Euler angle              
     Q     ( rad/sec )    pitch rate               
     pow                  power                    
     beta  ( rad )        sideslip angle           
     phi   ( rad )        Euler angle              
     P     ( rad/sec )    roll rate              
     R     ( rad/sec )    yaw rate ]           
       
 rdr  ( deg )         rudder deflection ]

system Output
y =[ an   ;q  ;alpha]
where 
an    ( ft/sec^2 )  - normal acceleration
 q    ( rad/sec )   - pitch rate
alpha ( rad )       - angle of attack

 Function Inputs
velocity ( ft/sec )- true velocity                    
altitude ( ft )    - altitude                         
xcg                - CoG fraction of mean aerodynamic chord 
 %}      
function [ A, B, C, D ] = ssm_f16( velocity, altitude, xcg, x_control) 

%---- trim the flight ----
[ state_trim, control_trim ] = find_trim( velocity, altitude, xcg );
state_dot_trim = zeros( length( state_trim ), 1 );
state_dot_trim(10) = velocity;
disp('  ');
disp( [ ' trimmed angle of attack    ( rad )     = ', num2str( state_trim(2) ) ] );
disp( [ ' trimmed sideslip angle     ( rad )     = ', num2str( state_trim(3) ) ] );
disp('  ');
disp( [ ' trimmed throttle           ( 0-1 )     = ', num2str( control_trim(1) ) ] );
disp( [ ' trimmed elevator           ( deg )     = ', num2str( control_trim(2) ) ] );
disp( [ ' trimmed aileron            ( deg )     = ', num2str( control_trim(3) ) ] );
disp( [ ' trimmed rudder             ( deg )     = ', num2str( control_trim(4) ) ] );
disp('  ');
% **************************state sapce Matrix***************************
% system state = [ Vt; h; alpha; theta; q; pow; beta; phi; p; r ];
% system control = [ delta_T; delta_E; delta_A; delta_R ];
% system output  = [ an - normal acceleration; q - pitch rate; alpha - angle of attack 
%---------------------------------------------------------
%   The linerization algorithm chooses smaller and      %
%   smaller perturbations in the independent variable   %
%   and compares three successive approximations to     %
%   the particular partial derivative. If they agree    %
%   within a certain tolerance, then the size of the    %
%   perturbation is reduced to determine if an even     %
%   smaller tolerance can be satisfied. The algorithm   %
%   terminates successfully when a tolerance TOLMIN     %
%   is reached.                                         %

x_control  = state_trim  ;
u_control = control_trim ;

%%                  1. velocity             ( ft/sec )       %%  
%%                  2. angle of attack      ( rad )          %%
%%                  3. sideslip angle       ( rad )          %%
%%                  4. phi   - Euler angle  ( rad )          %%
%%%                 5. theta - Euler angle  ( rad )          %%
%%%                 6. psi   - Euler angle  ( rad )          %%
%%                  7. roll rate            ( rad/sec )      %%
%%                  8. pitch rate           ( rad/sec )      %%
%%                  9. yaw rate             ( rad/sec )      %%
%%                  10.north displacement   ( ft )           %%
%%                  11.east displacement    ( ft )           %%    
%%                  12.altitude             ( ft )           %%
%%                  13.power  ( percent, 0 <= pow <= 100 ) ]

disp('state space matrices');
disp('-------------------------');

disp('system state(x_CONTROL)  = [ Vt; h; alpha; theta; q; pow; beta; phi; p; r ]')
disp('system control(u_control)= [ delta_T; delta_E; delta_A; delta_R ]');
A = ssm_a( state_trim, state_dot_trim, control_trim, xcg );
B = ssm_b( state_trim, state_dot_trim, control_trim, xcg );
C = ssm_c( state_trim, state_dot_trim, control_trim, xcg );
D = ssm_d( state_trim, state_dot_trim, control_trim, xcg );

