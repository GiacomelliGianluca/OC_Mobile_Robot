function theta_ref = direction (x_ref, y_ref, x_pos, y_pos)

if (x_ref >= x_pos && y_ref >= y_pos)
    theta_ref = atan((y_ref-y_pos)/(x_ref-x_pos));
elseif (x_ref <= x_pos && y_ref >= y_pos)
    theta_ref = atan((y_ref-y_pos)/(x_pos-x_ref)) + pi/2;
elseif (x_ref <= x_pos && y_ref <= y_pos)
    theta_ref = atan((y_pos-y_ref)/(x_pos-x_ref)) + pi;
else 
    theta_ref = atan((x_ref-x_pos)/(y_pos-y_ref)) - pi/2;
end

end
