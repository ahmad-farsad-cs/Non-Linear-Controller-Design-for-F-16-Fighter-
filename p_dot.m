% p_dot.m  rate of change of power
function power_dot = p_dot ( pow, cpow )
%cpow = command power level 
% pow = engine power level
if ( cpow >= 50.0 )
    if ( pow >= 50.0)
        tpow = cpow;
        t = 5.0;
    else
        tpow = 60.0;
        t = r_tc ( tpow - pow );
    end
else
    if ( pow >= 50.0)
        tpow = 40.0;
        t = 5.0;
    else
        tpow = cpow;
        t = r_tc ( tpow - pow );
    end
end 
    
power_dot = t * ( tpow - pow );