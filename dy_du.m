% dy_du.m partial derivative  y wrt u 
function  dy_du = dy_du( x, control, xcg, i, j, du )
time = 0.0;
t = control(j);
control(j) = t - du;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
y = [ an; q; alpha ];
y1 = y(i);            
control(j) = t + du;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
y = [ an; q; alpha ];
y2 = y(i);            
dy_du = ( y2 - y1 ) / ( du + du ); 
control(j) = t; 
