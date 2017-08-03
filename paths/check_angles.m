function check_angles(q) 

q1minmax = deg2rad(165);
q2minmax = deg2rad(110);
q3min = deg2rad(-110);
q3max = deg2rad(70);
q4minmax = deg2rad(160);
q5minmax = deg2rad(120);
q6minmax = deg2rad(400);

if abs(q(1)) > q1minmax
    disp(1);
    return;
end
if abs(q(2)) > q2minmax
    disp(2);
    return;
end
if q(3) < q3min || q(3) > q3max
    disp(3);
    return;
end
if abs(q(4)) > q4minmax
    disp(5);
    return;
end
if abs(q(5)) > q5minmax
    disp(5);
    return;
end
if abs(q(6)) > q6minmax
    disp(6);
    return;
end

if abs(q(7)) > q1minmax
    disp(7);
    return;
end
if abs(q(8)) > q2minmax
    disp(8);
    return;
end
if q(9) < q3min || q(9) > q3max
    disp(9);
    return;
end
if abs(q(10)) > q4minmax
    disp(10);
    return;
end
if abs(q(11)) > q5minmax
    disp(11);
    return;
end
if abs(q(12)) > q6minmax
    disp(12);
    return;
end


end