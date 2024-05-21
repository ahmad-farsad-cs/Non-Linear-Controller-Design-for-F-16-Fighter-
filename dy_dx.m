%dy_dx.m  partial derivative y wrt x
function  dy_dx = dy_dx( x, control, xcg, i, j, dx )
time = 0.0;
t = x(j);
x(j) = t - dx;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
y = [ an; q; alpha ];
y1 = y(i);      
x(j) = t + dx;  
[ x_dot, an, alat, qbar, amach, q, alpha ] = f16_dynam ( time, x, control, xcg );
y = [ an; q; alpha ];
y2 = y(i);      
dy_dx = ( y2 - y1 ) / ( dx + dx ); 
x(j) = t;  

