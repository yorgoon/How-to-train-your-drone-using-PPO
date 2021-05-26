function skew_mat = hat(vector3by1)
skew_mat = [0 -vector3by1(3) vector3by1(2);
            vector3by1(3) 0 -vector3by1(1);
            -vector3by1(2) vector3by1(1) 0];
end