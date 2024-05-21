
% ssm_A.m  state space matrix A for  nonlinear model
% x_dot = A * x + B * u
%   y   = C * x + D * u  

function A_reduce = ssm_A( state_eq, state_dot_eq, control_eq, xcg )

%-----------------------------------------
% ----  State space Matrix A--------------
%-----------------------------------------

%---- size of the Jacobian matrix ----
row = 1 : length(state_eq);    % array corresponding to rows of the A matrix
column = 1 : length(state_eq); % array corresponding to columns of the A matrix
no_row = length( row );        % No. of rows of the A matrix
no_col = length( column );     % No. of columns of the Amatrix

%---- parameters ----
del = 0.01;
dmin = 0.5;
tolmin = 3.3e-5;
oktol = 8.1e-4;
min_dv = 0.0001;

ij= 1;
answer = [];
count = 0;
for j = 1 : no_col
    dv0 = max( abs(del * state_eq(column(j)) ), dmin );
    dv1 = 1.1 * dv0;
    dv2 = 1.2 * dv0;
    abc = [];
    for i = 1 : no_row
        tol = 0.1;
        a0 = df_dx( state_eq, control_eq, xcg, row(i), column(j), dv0 );
        a1 = df_dx( state_eq, control_eq, xcg, row(i), column(j), dv1 );
        a2 = df_dx( state_eq, control_eq, xcg, row(i), column(j), dv2 );
        b0 = min( abs(a0), abs(a1) ); % b0 = min( |a0|, |a1| )
        b1 = min( abs(a1), abs(a2) );
        d0 = abs( a0 - a1 );          % d0 = | a1 - a0 |
        d1 = abs( a2 - a1 );
        k = 0;
        while k <= 100 
            new_dv = 0.6 * dv0;
            if new_dv <= min_dv * state_eq( column(j) )
                break;
            else
                a2 = a1;
                a1 = a0;
                a0 = df_dx( state_eq, control_eq, xcg, row(i), column(j), new_dv );
                b1 = b0;
                b0 = min( abs(a0), abs(a1) );
                d1 = d0;
                d0 = abs( a0 - a1 );
                if d0 <= tol * b0 & d1 <= tol * b1
                    tol = tol * 0.8;
                    if tol <= tolmin
                        tol = tolmin;
                        break;
                    end
                end
                k = k + 1;                    
            end
        end
        
        ans = a1;
        count = count + 1;
        abc = [ abc, ans ];
    end
    answer = [ answer;  abc];
end
ij = ij + 1;
answer = answer';
A_org = answer; 

%-----------------------------------------
%----      reorganize the A matrix    ----
%-----------------------------------------
% extracting rows 
answer_A = zeros( 10, 13 );
answer_A( 1, : ) = answer( 1 , : );
answer_A( 2, : ) = answer( 12, : );
answer_A( 3, : ) = answer( 2,  : );
answer_A( 4, : ) = answer( 5,  : );
answer_A( 5, : ) = answer( 8,  : );
answer_A( 6, : ) = answer( 13, : );
answer_A( 7, : ) = answer( 3,  : );
answer_A( 8, : ) = answer( 4,  : );
answer_A( 9, : ) = answer( 7,  : );
answer_A( 10,:)  = answer( 9,  : );

%% rearranging columns %%

answer = zeros( 10, 10 );
answer( :, 1 ) = answer_A( :, 1 );
answer( :, 2 ) = answer_A( :, 12 );
answer( :, 3 ) = answer_A( :, 2 );
answer( :, 4 ) = answer_A( :, 5 );
answer( :, 5 ) = answer_A( :, 8 );
answer( :, 6 ) = answer_A( :, 13 );
answer( :, 7 ) = answer_A( :, 3 );
answer( :, 8 ) = answer_A( :, 4 );
answer( :, 9 ) = answer_A( :, 7 );
answer( :, 10 ) = answer_A( :,9 );

%% ---- output of the reduced A matrix ----
A_reduce = answer;
% count;