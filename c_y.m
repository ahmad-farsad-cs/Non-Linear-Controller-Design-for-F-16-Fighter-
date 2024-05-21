
% c_y.m arrodynamics force in Y axis

function c_y_value = c_y( beta, ail, rdr )

c_y_value = -0.02 * beta + 0.021 * ( ail / 20.0 )+...
           0.086 * ( rdr / 30.0 );


