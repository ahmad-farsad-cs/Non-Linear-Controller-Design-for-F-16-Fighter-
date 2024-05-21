%df_du.m  partial derivative wrt to respective control input, using difference equation 
function  df_du = df_du( x, control, xcg, i, j, du )
time = 0.0;
t = control(j);
control(j) = t - du;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
xd1 = x_dot(i);       
control(j) = t + du;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
xd2 = x_dot(i);       
df_du = ( xd2 - xd1 ) / ( du + du ); 
control(j) = t;

