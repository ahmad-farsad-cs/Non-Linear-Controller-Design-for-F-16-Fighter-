% df_dx.m  partial derivative  wrt respecctive state difference equation
function  df_dx = df_dx( x, control, xcg, i, j, dx )
time = 0.0;
t = x(j);
x(j) = t - dx;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
xd1 = x_dot(i); 
x(j) = t + dx;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
xd2 = x_dot(i); 
df_dx = ( xd2 - xd1 ) / ( dx + dx ); 
x(j) = t;

