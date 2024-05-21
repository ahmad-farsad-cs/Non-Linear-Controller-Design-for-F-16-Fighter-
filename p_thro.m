% P_thro.m 
function p_thro_value  = p_thro ( thro )
if ( thro <= 0.77 )
    p_thro_value = 64.94 * thro;
else
    p_thro_value = 217.38 * thro - 117.38;
end

    