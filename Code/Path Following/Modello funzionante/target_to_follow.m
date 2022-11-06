function [x_target, y_target, index_new] = target_to_follow(reference, pos_x, pos_y, index)

current_target_x = reference(1,index);
current_target_y = reference(1,index);

distance = sqrt((current_target_x - pos_x)^2+(current_target_y - pos_y)^2);

if distance <= 1
    index_new = index + 1;
else
    index_new = index;
end


x_target = reference(1,index_new);
y_target = reference(1,index_new);


end