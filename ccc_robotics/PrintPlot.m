function [ ] = PrintPlot( plt, name)

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');


figure(2);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

figure(4);
hplot = plot(plt.t, plt.a(8:15,:));
set(hplot, 'LineWidth', 2);
legend('Amu', 'Aha', 'Amav', 'Al', 'Aalr', 'AvNull','At', 'Av');

figure(5);
if (strcmp(name,'Robust' ))
    hplot = plot(plt.t, plt.Error(:,:));
    set(hplot, 'LineWidth', 2);
    legend('normVehicleError', 'normAlignmentError', 'normToolFrameError');
    title ('Error');    
else %DEXROV
    hplot = plot(plt.t, plt.toolFrameError(:,:));
    set(hplot, 'LineWidth', 2);
    legend('ErrorWx', 'ErrorWy', 'ErrorWz', 'ErrorX', 'ErrorY', 'ErrorZ');
    title ('Error with TPIK2');
end
% Plot for joint limits
figure(6);
for i = 1:7
    subplot(3,3,i);
    plot(plt.t, plt.q(i,:),'LineWidth', 1,'color','yellow');
    hold on;
    plot(plt.t, plt.jlmin(i,:), '--','LineWidth', 0.5,'color','blue');
    hold on;
    plot(plt.t, plt.jlmax(i,:), '--','LineWidth', 0.5,'color','red');
    title (['Joint ', num2str(i)]);
end



end

