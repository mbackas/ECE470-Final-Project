function b = collision(p_1, p_2, r_1, r_2)
    diff = p_1 - p_2;
    dist = diff(1)^2 + diff(2)^2 + diff(3)^2;
    if(sqrt(dist) < r_1+r_2)
       b = true;
       fprintf('%f < %f\n', sqrt(dist), (r_1+r_2));
    else
        b = false;
    end
end