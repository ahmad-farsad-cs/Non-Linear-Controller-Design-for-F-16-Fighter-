% N_R.m  yawing moment due to rudder

function n_r_value = n_r ( alpha, beta )
A = [ -.018  -.052  -.052  -.052  -.054  -.049  -.059  -.051  -.030  -.037  -.026  -.013;
      -.028  -.051  -.043  -.046  -.045  -.049  -.057  -.052  -.030  -.033  -.030  -.008;
      -.037  -.041  -.038  -.040  -.040  -.038  -.037  -.030  -.027  -.024  -.019  -.013; 
      -.048  -.045  -.045  -.045  -.044  -.045  -.047  -.048  -.049  -.045  -.033  -.016;
      -.043  -.044  -.041  -.041  -.040  -.038  -.034  -.035  -.035  -.029  -.022  -.009;
      -.052  -.034  -.036  -.036  -.035  -.028  -.024  -.023  -.020  -.016  -.010  -.014;
      -.062  -.034  -.027  -.028  -.027  -.027  -.023  -.023  -.019  -.009  -.025  -.010 ];
A = A';  
row = 3;
col = 4;
s = 0.2 * alpha;
k = fix ( s );
if ( k <= -2 )
    k = -1;
end
if ( k >= 9 )
    k = 8;
end
da = s -  k ;
l = k + fix ( sign ( da ) * 1.1 );
%boundary condition
if l < -2
    l = -2;
elseif l > 9
        l = 9;
end
s = 0.1 * beta;
m = fix ( s );
if ( m <= -3 )
   m = -2;
end
if ( m >= 3)
   m = 2;
end
db = s -  m ;
n = m + fix ( sign ( db ) * 1.1 );
% boundary condition
if n < -3
    n = -3;
elseif n > 3
        n = 3;
end
t = A( k+row, m+col );
u = A( k+row, n+col );
v = t + abs( da ) * ( A( l+row, m+col ) - t );
w = u + abs( da ) * ( A( l+row, n+col ) - u );
n_r_value = v + ( w - v ) * abs( db );



