function image = features_to_image(features)

%     image(:, :, 1) = double(features.environment | features.start_car.arrow | features.end_car.arrow);
%     image(:, :, 1) = double(features.environment | features.start_car.triangle | features.end_car.triangle);
    image(:, :, 1) = double(features.environment);
    image(:, :, 2) = double(features.start_car.triangle);
    image(:, :, 3) = double(features.end_car.triangle);

end