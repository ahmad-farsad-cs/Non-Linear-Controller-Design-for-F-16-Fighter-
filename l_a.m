%  l_a.m rolling moment due to ailerons 

function l_a_value = l_a ( alpha, beta )
A = [ -.041  -.052  -.053  -.056  -.050  -.056  -.082  -.059  -.042  -.038  -.027  -.017;
      -.041  -.053  -.053  -.053  -.050  -.051  -.066  -.043  -.038  -.027  -.023  -.016;
      -.042  -.053  -.052  -.051  -.049  -.049  -.043  -.035  -.026  -.016  -.018  -.014;
      -.040  -.052  -.051  -.052  -.048  -.048  -.042  -.037  -.031  -.026  -.017  -.012;
      -.043  -.049  -.048  -.049  -.043  -.042  -.042  -.036  -.025  -.021  -.016  -.011;
      -.044  -.048  -.048  -.047  -.042  -.041  -.020  -.028  -.013  -.014  -.011  -.010;
      -.043  -.049  -.047  -.045  -.042  -.037  -.003  -.013  -.010  -.003  -.007  -.008 ];
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
% boundary condition
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
l_a_value = v + ( w - v ) * abs( db );



