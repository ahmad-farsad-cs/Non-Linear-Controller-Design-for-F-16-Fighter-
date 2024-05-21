%  c_z.m          aerodynamic force in z-axis %% 
function c_z_value = c_z( alpha, beta, el )

A = [ 0.770  0.241  -0.100  -0.416  -0.731  -1.053  -1.366  -1.646  -1.917  -2.120  -2.248  -2.229 ];
A = A';  
row = 3;

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
%boundary condition
if l < -2
    l = -2;
elseif l > 9
        l = 9;
end
s = A( k+row ) + abs( da ) * ( A( l+row ) - A( k+row ) );
c_z_value = s * ( 1 - ( beta / 57.3 ) ^ 2 ) - 0.19 * ( el / 25.0 );


