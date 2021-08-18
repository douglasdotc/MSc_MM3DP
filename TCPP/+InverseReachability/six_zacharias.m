function poses= six_zacharias(radius,n6)
if mod(n6,6)~=0
    disp("n must be a multiple of 6.")
    assert(mod(n6,6)==0);    
end

n6=n6/2;
points = SpiralSampleSphere(n6) * radius;
R = eul2rotm([atan2(points(:, 2), points(:, 1)), -atan2(points(:, 3), sqrt(sum(points(:, 1:2).^2, 2))), zeros(n6, 1)]);
R = rotm2tform(R);
R = TForm.tformX(R, rotm2tform(eul2rotm([0 pi / 2 0], 'ZYZ')));
R_out = TForm.tformX(R, eul2tform([linspace(0, 2 * pi, n6)' zeros(n6, 1) zeros(n6, 1)], 'ZYZ'));

R_1 = TForm.tformX(R, eul2tform([0 pi/2 0], 'ZYZ'));
R_2 = TForm.tformX(R, eul2tform([pi/2 pi/2 0], 'ZYZ'));
R_3 = TForm.tformX(R, eul2tform([-pi/2 pi/2 0], 'ZYZ'));
R_4 = TForm.tformX(R, eul2tform([pi pi/2 0], 'ZYZ'));
R_in = TForm.tformX(R, eul2tform([0 pi 0], 'ZYZ'));

poses = [points tform2quat(R_out);
    points tform2quat(R_1);
    points tform2quat(R_2);
    points tform2quat(R_3);
    points tform2quat(R_4);
    points tform2quat(R_in);
    ];

end


