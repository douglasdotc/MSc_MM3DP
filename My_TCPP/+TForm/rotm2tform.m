function T = DOWN()
    T = [eul2rotm([0 pi 0], 'ZYZ'), zeros(3, 1); zeros(1, 3) 1];
