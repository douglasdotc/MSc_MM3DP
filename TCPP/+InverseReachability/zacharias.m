function poses= zacharias(radius,n)

points = SpiralSampleSphere(n) * radius;
R = eul2rotm([atan2(points(:, 2), points(:, 1)), -atan2(points(:, 3), sqrt(sum(points(:, 1:2).^2, 2))), zeros(n, 1)]);
R = rotm2tform(R);
R = TForm.tformX(R, rotm2tform(eul2rotm([0 pi / 2 0], 'ZYZ')));
R = TForm.tformX(R, eul2tform([linspace(0, 2 * pi, n)' zeros(n, 1) zeros(n, 1)], 'ZYZ'));
poses = [points tform2quat(R)];
end

