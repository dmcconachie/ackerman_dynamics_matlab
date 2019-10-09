function save_to_file(features, filepath)

image(:, :, 1) = double(features.environment);
image(:, :, 2) = double(features.start_car);
image(:, :, 3) = double(features.end_car);
imwrite(image, filepath)

end