function h = mush_shape(width,length,t)

shape=[0 0 0;
-1 0 1;
-1 -1 2;
1  -1 3;
1 0 4;
0 0 5;
0 1 6;
2 1 7;
2 0 8;
0 0 9];

shape(:,1)=length*shape(:,1);
shape(:,2)=width*shape(:,2);
shape(:,3)=shape(:,3)/9;
h=interp1(shape(:,3),shape(:,1:2),t);


end

