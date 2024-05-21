%c_n.m  yawing moment coefficients
function c_n_value = c_n( alpha, beta )
A = [ .0    .0    .0    .0    .0    .0    .0    .0    .0     .0     .0     .0  ;
      .018  .019  .018  .019  .019  .018  .013  .007  .004  -.014  -.017  -.033;
      .038  .042  .042  .042  .043  .039  .030  .017  .004  -.035  -.047  -.057;
      .056  .057  .059  .058  .058  .053  .032  .012  .002  -.046  -.071  -.073;
      .064  .077  .076  .074  .073  .057  .029  .007  .012  -.034  -.065  -.041;
      .074  .086  .093  .089  .080  .062  .049  .022  .028  -.012  -.002  -.013;
      .079  .090  .106  .106  .096  .080  .068  .030  .064   .015   .011  -.001 ];
A = A';  
row = 3;
col = 1;
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
s = 0.2 * abs( beta );
m = fix ( s );
if ( m == 0 )
   m = 1;
end
if ( m >= 6)
   m = 5;
end
db = s - m;
n = m + fix ( sign ( db ) * 1.1 );
% boundary condition
if n < 0
    n = 0;
elseif n > 6
        n = 6;
end
t = A( k+row, m+col );
u = A( k+row, n+col );
v = t + abs( da ) * ( A( l+row, m+col ) - t );
w = u + abs( da ) * ( A( l+row, n+col ) - u );
dum = v + ( w - v ) * abs( db );
c_n_value = dum * sign ( beta ) * 1.0;
