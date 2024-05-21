%{
*********runme.m**********
%}
close all;
clear all;
clc;
%{
disp( 'Enter 1. To linearize aircraft model  ' );
disp( 'Enter 2. To simulate the exixting nonlinear model' );
disp( '  ' );
index = input( ' which task do you wish to accomplish? ' );
disp( '  ' );
	switch index
		case 1
            disp('  ');
            disp(' You have chosen  linearze model. ');
            disp('  ');
            disp(' linearized model : x_dot = A * x + B * u' );
            disp('                      y   = C * x + D * u');
            disp('  ');
            disp('  ------system States vector  --------                  ');
            disp(' x= [ vt; h; alpha; theta; Q; pow; beta; phi; P: R] ');
            disp('     where x(1) -vt    ( ft/sec )    - velocity               ');
            disp('           x(2) -h     ( ft )        - altitude               ');
            disp('           x(3) -alpha ( rad )       - angle of attack        ');  
            disp('           x(4) -theta ( rad )       - Euler angle            ');  
            disp('           x(5) -Q     ( rad/sec )   - pitch rate             ');  
            disp('           x(6) -pow                 - power                  ');  
            disp('           x(7) -beta  ( rad )       - sideslip angle         ');  
            disp('           x(8) -phi   ( rad )       - Euler angle            ');  
            disp('           x(9) -P     ( rad/sec )   - roll rate              ');  
            disp('           x(10)-R     ( rad/sec )   - yaw rate ];            ');  
            disp('                                                        ');  
            disp('  ---- system Control vector --------                   ');
            disp('   u = [ thtl ;el  ;ail   ;rdr]                         ');
            disp('   where  thtl ( 0 ~ 1.0 )    - throttle setting        ');  
            disp('           u(1) -el   ( deg )        - elevator deflection    ');  
            disp('           u(2) -ail  ( deg )        - aileron deflection     ');  
            disp('           u(3) -rdr  ( deg )        - rudder deflection ];   ');  
            disp('                                                        ');  
        
            disp('  ---- Output  vector( first 4 longitidunal last 6 lateral -------- ');    
            disp(' y= [ vt; h; alpha; theta; Q; pow; beta; phi; P: R] ');
            disp('     where y(1) -vt    ( ft/sec )    - velocity               ');
            disp('           y(2) -h     ( ft )        - altitude               ');
            disp('           y(3) -alpha ( rad )       - angle of attack        ');  
            disp('           y(4) -theta ( rad )       - Euler angle            ');  
            disp('           y(5) -Q     ( rad/sec )   - pitch rate             ');  
            disp('           y(6) -pow                 - power                  ');  
            disp('           y(7) -beta  ( rad )       - sideslip angle         ');  
            disp('           y(8) -phi   ( rad )       - Euler angle            ');  
            disp('           y(9) -P     ( rad/sec )   - roll rate              ');  
            disp('           y(10)-R     ( rad/sec )   - yaw rate ];            '); 
          
            disp('  ');
            disp('  ');
            disp(' please enter flight parameter calculate the optimized trim point using mimmumt cost function .');
            disp('  ');
            velocity = input(' true velocity ( 350 ~ 900 ft/sec ) : ');
            disp('  ');
            altitude = input(' altitude ( 0 ~ 40000 ft ) : ');
            disp('  ');
            xcg = input(' center of gravity position ( 0.2 ~ 0.5, reference xcg = 0.35 )  ');
            disp('  ');
            [ A, B, C, D ] = ssm_f16( velocity, altitude, xcg )
            %[num1 den1] = ss2tf(A,B,C,D,1)  % iu = 1
            %[num2 den2] = ss2tf(A,B,C,D,2)  % iu = 2
            sys = ss(A,B,C,D);
            TF = tf(sys);
            disp(' transfer function matrix (TF) gives dynamics of the airplane in the horizontal');
            disp(' TF matrix is [3*4] order ');
            disp(' where output = y = [ an; q; alpha ] and input is T.E.A.R. ');
            rlocus(TF(1,1))
            disp(' state stability anaysis ');
            disp(' taking states as output , where first 4 elememts are longitudnal and next 6 are lateral parameters ');
            disp(' state stability anaysis calculating states as output new C and D matrix will be  ');
            C_1 =eye(10);
            D_1 =zeros(10,4);
            sys_x = ss(A,B,C_1,D_1);
            TF_x = tf(sys_x);
			for i=1:10
                figure(i);
                hold;
			for j=1:4
                    subplot(1,4,j);
                    pzmap(TF_x(i,j));
			end
			end
			poles = pole(sys); 
% two unstable poles . 
% assuming desired poles at 
% two unstable poles at = P 
			P_d= [-17, -9, -5, -5, -5 ,-4, -3, -4,-6,-1];

% full state feedback matrix
			K=place(A,B,P_d); 

		case 2
            %}
			disp('  ');
            % disp(' simulating of nonlinear model. ');
            disp('  ');
            disp(' To simulate the aircraft, please choose the flight conditions to calculate optimized trim conditions');
            disp(' The control variables are left as inputs to the block of  nonlinear model');
            disp(' Please input the following flight parameters:');
            disp('  ');
            velocity = input(' true velocity ( 350 ~ 1000 ft/sec ) : ');
            disp('  ');
            altitude = input(' altitude ( 0 ~ 40000 ft ) : ');
            disp('  ');
            xcg = input(' center of gravity position ( 0.2 ~ 0.5, reference xcg = 0.35 )  ');
            disp('  ');            
            [ state_trim, control_trim ] = find_trim( velocity, altitude, xcg )                 ;
            disp('  ');

            disp( [ ' trimmed angle of attack    ( rad )     = ', num2str( state_trim(2) ) ] )  ;
            disp( [ ' trimmed sideslip angle     ( rad )     = ', num2str( state_trim(3) ) ] )  ;
            disp('  ');
            disp( [ ' trimmed throttle           ( 0-1 )     = ', num2str( control_trim(1) ) ] );
            disp( [ ' trimmed elevator           ( deg )     = ', num2str( control_trim(2) ) ] );
            disp( [ ' trimmed aileron            ( deg )     = ', num2str( control_trim(3) ) ] );
            disp( [ ' trimmed rudder             ( deg )     = ', num2str( control_trim(4) ) ] );
            disp('  ');
            disp(' transfer function matrix (TF) gives dynamics of the airplane in the horizontal');
            disp(' TF matrix is [3*4] order ');
            disp(' where output = y = [ an; q; alpha ] and input is T.E.A.R. ');
            
%            rlocus(TF(1,1)) ;

            disp(' state stability anaysis ');
            disp(' taking states as output , where first 4 elememts are longitudnal and next 6 are lateral parameters, 11th state is accelerastion ');
            %disp(' state stability anaysis calculating states as output new C and D matrix will be  ');
            [ A, B, C, D ] = ssm_f16( velocity, altitude, xcg );
            C_1 =eye(10);
            D_1 =zeros(10,4); 
            sys_y = ss(A,B,C,D);
            TF_y = tf(sys_y);
            sys_x = ss(A,B,C_1,D_1);
            TF_x = tf(sys_x);
            [num_y_2,den_y_2] = tfdata(TF_y(2,2)) ; num_y_2=cell2mat(num_y_2);den_y_2=cell2mat(den_y_2);
            [num_x_8,den_x_8] = tfdata(TF_x(8,3)) ;num_x_8=cell2mat(num_x_8);den_x_8=cell2mat(den_x_8);
            [num_x_7,den_x_7] = tfdata(TF_x(7,4)) ;num_x_7=cell2mat(num_x_7);den_x_7=cell2mat(den_x_7);
            [num_x_2,den_x_2] = tfdata(TF_x(2,2)) ;num_x_2=cell2mat(num_x_2);den_x_2=cell2mat(den_x_2);
            [num_y_1,den_y_1] = tfdata(TF_y(1,1)) ;num_y_1=cell2mat(num_y_1);den_y_1=cell2mat(den_y_1);
            [num_y_3,den_y_3] = tfdata(TF_y(3,2)) ;num_y_3=cell2mat(num_y_3);den_y_3=cell2mat(den_y_3);

            disp( [ 'now you can simulate state space model' ] );

            k = input(' enter any key to continue ');
            clear k

            sim( 'simF16_openloop', [ 0, 10 ]);
            % plot the figures
            figure(1);
            subplot(2,2,1); plot(Time, simout(:,1)); title('velocity  vs.  time'); grid;
            ylabel(' velocity (ft/sec)'); xlabel('time (sec)');
            subplot(2,2,2); plot(Time, simout(:,2)); title('angle of attack  vs.  time'); grid;
            ylabel(' angle of attack (rad)'); xlabel('time (sec)');
            subplot(2,2,3); plot(Time, simout(:,3)); title('sideslip angle  vs.  time'); grid;
            ylabel(' sideslip angle (rad)'); xlabel('time (sec)');
            subplot(2,2,4); plot(Time, simout(:,16)); title('mach number  vs.  time'); grid;
            ylabel(' mach number '); xlabel('time (sec)');
            figure(2);   
            subplot(3,1,1); plot(Time, simout(:,4)); title('euler angle phi  vs.  time'); grid;
            ylabel(' phi (rad)'); xlabel('time (sec)');
            subplot(3,1,2); plot(Time, simout(:,5)); title('euler angle theta  vs.  time'); grid;
            ylabel(' theta (rad)'); xlabel('time (sec)');
            subplot(3,1,3); plot(Time, simout(:,6)); title('euler angle psi  vs.  time'); grid;
            ylabel(' psi (rad)'); xlabel('time (sec)');
            figure(3);   
            subplot(3,1,1); plot(Time, simout(:,7)); title('roll rate  vs.  time'); grid;
            ylabel(' roll rate (rad/sec)'); xlabel('time (sec)');            
            subplot(3,1,2); plot(Time, simout(:,8)); title('pitch rate  vs.  time'); grid;
            ylabel(' pitch rate (rad/sec)'); xlabel('time (sec)');
            subplot(3,1,3); plot(Time, simout(:,9)); title('yaw rate  vs.  time'); grid;
            ylabel(' yaw rate (rad/sec)'); xlabel('time (sec)');
            figure(4);  
            subplot(2,2,1); plot(Time, simout(:,10)); title('north distance  vs.  time'); grid;
            ylabel(' north distance (ft)'); xlabel('time (sec)');
            subplot(2,2,2); plot(Time, simout(:,11)); title('east distance  vs.  time'); grid;
            ylabel(' east distance (ft)'); xlabel('time (sec)');
            subplot(2,2,3); plot(Time, simout(:,12)); title('altitude  vs.  time'); grid;
            ylabel(' altitude (ft)'); xlabel('time (sec)');
            subplot(2,2,4); plot(Time, simout(:,13)); title('normal acceleration  vs.  time'); grid;
            ylabel(' normal accelaration (ft/sec^2)'); xlabel('time (sec)');  
%	end
	
disp( 'Simulation Done.' );
disp( ' please refer to figures for results' );
disp( '  ' );
