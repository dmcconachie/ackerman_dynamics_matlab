function plot_env(ex1, ex2)

%% First display the lines representing the entire trajectories
%% Ex1
subplot(2, 3, 1)
title(sprintf('Slow start v = %g', ex1.slow.x0(end)))
plot_obstacle(ex1.obstacle);
hold on
plot(ex1.lifted_y_ref(1, :), ex1.lifted_y_ref(2, :), ex1.ref_color);
if isfield(ex1.slow, 'x_history')
    plot(ex1.slow.x_history(1, :), ex1.slow.x_history(2, :), ex1.slow.color);
end
axis equal
hold off

subplot(2, 3, 2)
title(sprintf('Medium start v = %g', ex1.med.x0(end)))
plot_obstacle(ex1.obstacle);
hold on
plot(ex1.lifted_y_ref(1, :), ex1.lifted_y_ref(2, :), ex1.ref_color);
if isfield(ex1.med, 'x_history')
    plot(ex1.med.x_history(1, :), ex1.med.x_history(2, :), ex1.med.color);
end
axis equal
hold off

subplot(2, 3, 3)
title(sprintf('Fast start v = %g', ex1.fast.x0(end)))
plot_obstacle(ex1.obstacle);
hold on
plot(ex1.lifted_y_ref(1, :), ex1.lifted_y_ref(2, :), ex1.ref_color);
if isfield(ex1.fast, 'x_history')
    plot(ex1.fast.x_history(1, :), ex1.fast.x_history(2, :), ex1.fast.color);
end
axis equal
hold off

%% Ex2
subplot(2, 3, 4)
title(sprintf('Slow start v = %g', ex2.slow.x0(end)))
plot_obstacle(ex2.obstacle);
hold on
plot(ex2.lifted_y_ref(1, :), ex2.lifted_y_ref(2, :), ex2.ref_color);
if isfield(ex2.slow, 'x_history')
    plot(ex2.slow.x_history(1, :), ex2.slow.x_history(2, :), ex2.slow.color);
end
axis equal
hold off

subplot(2, 3, 5)
title(sprintf('Medium start v = %g', ex2.med.x0(end)))
plot_obstacle(ex2.obstacle);
hold on
plot(ex2.lifted_y_ref(1, :), ex2.lifted_y_ref(2, :), ex2.ref_color);
if isfield(ex2.med, 'x_history')
    plot(ex2.med.x_history(1, :), ex2.med.x_history(2, :), ex2.med.color);
end
axis equal
hold off

subplot(2, 3, 6)
title(sprintf('Fast start v = %g', ex2.fast.x0(end)))
plot_obstacle(ex2.obstacle);
hold on
plot(ex2.lifted_y_ref(1, :), ex2.lifted_y_ref(2, :), ex2.ref_color);
if isfield(ex2.fast, 'x_history')
    plot(ex2.fast.x_history(1, :), ex2.fast.x_history(2, :), ex2.fast.color);
end
axis equal
hold off

end