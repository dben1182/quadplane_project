deltas = csvread('deltas.csv');
totalState = csvread('TotalState.csv');
Currents = csvread('Current.csv');


%gets the magnitude of the velocity
velocities = totalState(4:6,:);
x_vel = velocities(1,:);
y_vel = velocities(2,:);
z_vel = velocities(3,:);

%iterates through and gets the magnitude of the velocities
velocitiesLength = length(velocities(1,:));
velocityMagnitude = zeros(velocitiesLength);

for i = 1:velocitiesLength
    velocityMagnitude(i) = sqrt(x_vel(i)^2 + y_vel(i)^2 + z_vel(i)^2);
end

figure(1);
plot(x_vel);
hold on;
plot(y_vel);
plot(z_vel);
plot(velocityMagnitude);
hold off;
title("Velocities");
legend('x', 'y', 'z', 'Magnitude');



figure(2);
plot(deltas(1,:));
hold on;
plot(deltas(2,:));
plot(deltas(3,:));
plot(deltas(4,:));
hold off;
title('Vertical Deltas');
legend('V1', 'V2', 'V2', 'V4');


figure(3);
plot(deltas(5,:));
hold on;
plot(deltas(6,:));
plot(deltas(7,:));
plot(deltas(8,:));
hold off;
title('Control Surface Deltas');
legend('throttle', 'elevator', 'aileron', 'rudder');



figure(4);
plot(Currents(1,:));
hold on;
plot(Currents(2,:));
plot(Currents(3,:));
plot(Currents(4,:));
hold off;
title('Currents');
legend('I1', 'I2', 'I3', 'I4');