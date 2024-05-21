% L_R.m  rolling moment due to rudder 
function l_r_value = l_r ( alpha, beta )
A = [ .005  .017  .014  .010  -.005  .009  .019  .005   .000  -.005  -.011  .008;
      .007  .016  .014  .014   .013  .009  .012  .005   .000   .004   .009  .007;
      .013  .013  .011  .012   .011  .009  .008  .005   .000   .005   .003  .005;
      .018  .015  .015  .014   .014  .014  .014  .015   .013   .011   .006  .001;
      .015  .014  .013  .013   .012  .011  .011  .010   .008   .008   .007  .003;
      .021  .011  .010  .011   .010  .009  .008  .010   .006   .005   .000  .001;
      .023  .010  .011  .011   .011  .010  .008  .010   .006   .014   .020  .000 ];
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
l_r_value = v + ( w - v ) * abs( db );

