deltas = csvread('outputs/deltas.csv');
totalState = csvread('outputs/totalState.csv');



figure(1);
plot(deltas(5,:));
title('Throttle forward');

figure(2);
plot(deltas(6,:));
title('Elevator');

figure(3);
plot(deltas(7,:));
title('Aileron');

figure(4);
plot(deltas(8,:));
title('Rudder');

velocities = totalState(4:6,:);

figure(5);
plot(velocities(1,:));
hold on;
plot(velocities(2,:));
plot(velocities(3,:));
hold off;
title("velocities")


