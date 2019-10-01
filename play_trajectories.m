function play_trajectories(ex1, ex2)

% Iterate through the states, displaying each in turn
for idx = 1:size(ex1.y_ref, 2)
    clf
    plot_env(ex1, ex2);
    
    subplot(2, 3, 1);
    hold on; 
    plot_car(ex1.slow.x_history(1, idx), ...
             ex1.slow.x_history(2, idx), ...
             ex1.slow.x_history(3, idx), ...
             ex1.slow.x_history(4, idx), ...
             ex1.l, ex1.w, ex1.slow.color);
    
    subplot(2, 3, 2);
    hold on; 
    plot_car(ex1.med.x_history(1, idx), ...
             ex1.med.x_history(2, idx), ...
             ex1.med.x_history(3, idx), ...
             ex1.med.x_history(4, idx), ...
             ex1.l, ex1.w, ex1.med.color);
    
    subplot(2, 3, 3);
    hold on; 
    plot_car(ex1.fast.x_history(1, idx), ...
             ex1.fast.x_history(2, idx), ...
             ex1.fast.x_history(3, idx), ...
             ex1.fast.x_history(4, idx), ...
             ex1.l, ex1.w, ex1.fast.color);
         
    subplot(2, 3, 4);
    hold on; 
    plot_car(ex2.slow.x_history(1, idx), ...
             ex2.slow.x_history(2, idx), ...
             ex2.slow.x_history(3, idx), ...
             ex2.slow.x_history(4, idx), ...
             ex2.l, ex2.w, ex2.slow.color);
    
    subplot(2, 3, 5);
    hold on; 
    plot_car(ex2.med.x_history(1, idx), ...
             ex2.med.x_history(2, idx), ...
             ex2.med.x_history(3, idx), ...
             ex2.med.x_history(4, idx), ...
             ex2.l, ex2.w, ex2.med.color);
    
    subplot(2, 3, 6);
    hold on; 
    plot_car(ex2.fast.x_history(1, idx), ...
             ex2.fast.x_history(2, idx), ...
             ex2.fast.x_history(3, idx), ...
             ex2.fast.x_history(4, idx), ...
             ex2.l, ex2.w, ex2.fast.color);
    
    drawnow;
    pause(0.01);
end

end

%%
function plot_env(ex1, ex2)

%% First display the lines representing the entire trajectories
%% Ex1
subplot(2, 3, 1)
title(sprintf('Slow start v = %g', ex1.slow.x0(end)))
plot_obstacle(ex1.obstacle);
hold on
plot(ex1.lifted_y_ref(1, :), ex1.lifted_y_ref(2, :), ex1.ref_color);
plot(ex1.slow.x_history(1, :), ex1.slow.x_history(2, :), ex1.slow.color);
axis equal
hold off

subplot(2, 3, 2)
title(sprintf('Medium start v = %g', ex1.med.x0(end)))
plot_obstacle(ex1.obstacle);
hold on
plot(ex1.lifted_y_ref(1, :), ex1.lifted_y_ref(2, :), ex1.ref_color);
plot(ex1.med.x_history(1, :), ex1.med.x_history(2, :), ex1.med.color);
axis equal
hold off

subplot(2, 3, 3)
title(sprintf('Fast start v = %g', ex1.fast.x0(end)))
plot_obstacle(ex1.obstacle);
hold on
plot(ex1.lifted_y_ref(1, :), ex1.lifted_y_ref(2, :), ex1.ref_color);
plot(ex1.fast.x_history(1, :), ex1.fast.x_history(2, :), ex1.fast.color);
axis equal
hold off

%% Ex2
subplot(2, 3, 4)
title(sprintf('Slow start v = %g', ex2.slow.x0(end)))
plot_obstacle(ex2.obstacle);
hold on
plot(ex2.lifted_y_ref(1, :), ex2.lifted_y_ref(2, :), ex2.ref_color);
plot(ex2.slow.x_history(1, :), ex2.slow.x_history(2, :), ex2.slow.color);
axis equal
hold off

subplot(2, 3, 5)
title(sprintf('Medium start v = %g', ex2.med.x0(end)))
plot_obstacle(ex2.obstacle);
hold on
plot(ex2.lifted_y_ref(1, :), ex2.lifted_y_ref(2, :), ex2.ref_color);
plot(ex2.med.x_history(1, :), ex2.med.x_history(2, :), ex2.med.color);
axis equal
hold off

subplot(2, 3, 6)
title(sprintf('Fast start v = %g', ex2.fast.x0(end)))
plot_obstacle(ex2.obstacle);
hold on
plot(ex2.lifted_y_ref(1, :), ex2.lifted_y_ref(2, :), ex2.ref_color);
plot(ex2.fast.x_history(1, :), ex2.fast.x_history(2, :), ex2.fast.color);
axis equal
hold off

end