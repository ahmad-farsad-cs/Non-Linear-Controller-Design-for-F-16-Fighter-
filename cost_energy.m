% cost_energy.m
function cost = cost_energy( state, control, s, xcg, weight )

% state   - (Xd floating states from dynmic model ( to be optimized ) 
% control - current control
%  s      - current steady state
control = [ s(1); s(2); s(4); s(5) ];
state(2) = s( 3 );
state(3) = s( 6 );
state(13) = p_thro( control(1) );
% steady state parameters
state_ss = steady_state( state ); % steady state 
time = 0;
% global xcg;
[ xd, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, state_ss, control, xcg );
cost = weight*(xd.*xd);

