
% n_a.m yawing moment due to ailerons 
function n_a_value = n_a ( alpha, beta )

A = [  .001  -.027  -.017  -.013  -.012  -.016   .001   .017   .011  .017   .008  .016
       .002  -.014  -.016  -.016  -.014  -.019  -.021   .002   .012  .016   .015  .011
      -.006  -.008  -.006  -.006  -.005  -.008  -.005   .007   .004  .007   .006  .006
      -.011  -.011  -.010  -.009  -.008  -.006   .000   .004   .007  .010   .004  .010
      -.015  -.015  -.014  -.012  -.011  -.008  -.002   .002   .006  .012   .011  .011
      -.024  -.010  -.004  -.002  -.001   .003   .014   .006  -.001  .004   .004  .006
      -.022   .002  -.003  -.005  -.003  -.001  -.009  -.009  -.001  .003  -.002  .001  ];
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
n_a_value = v + ( w - v ) * abs( db );



