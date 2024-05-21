%c_x.m aerodynamic force in x axis coeff.

function c_x_value = c_x( alpha, el )

A = [ -0.099  -0.081  -0.081  -0.063  -0.025  0.044  0.097  0.113  0.145  0.167  0.174  0.166;
      -0.048  -0.038  -0.040  -0.021   0.016  0.083  0.127  0.137  0.162  0.177  0.179  0.167;
      -0.022  -0.020  -0.021  -0.004   0.032  0.094  0.128  0.130  0.154  0.161  0.155  0.138;
      -0.040  -0.038  -0.039  -0.025   0.006  0.062  0.087  0.085  0.100  0.110  0.104  0.091;
      -0.083  -0.073  -0.076  -0.072  -0.046  0.012  0.024  0.025  0.043  0.053  0.047  0.040 ];
A = A';  
row = 3;
col = 3;
s = 0.2 * alpha;
k = fix ( s );
if ( k <= -2 )
    k = -1;
end
if ( k >= 9 )
    k = 8;
end
da = s - k;
l = k + fix ( sign ( da ) * 1.1 );
% boundary condition
if l < -2
    l = -2;
elseif l > 9
        l = 9;
end
s = el / 12.0;
m = fix ( s );
if ( m <= -2 )
   m = -1;
end
if ( m >= 2)
   m = 1;
end
de = s - m;
n = m + fix ( sign ( de ) * 1.1 );
% boundary condition
if n < -2
    n = -2;
elseif n > 2
        n = 2;
end
t = A( k+row, m+col );
u = A( k+row, n+col );
v = t + abs( da ) * ( A( l+row, m+col ) - t );
w = u + abs( da ) * ( A( l+row, n+col ) - u );
c_x_value = v + ( w - v ) * abs( de );


