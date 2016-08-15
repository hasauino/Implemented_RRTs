function []=move2(p,x,y,z,th)
 global orig
 rot = angle2dcm(th,0,0); %Create a rotation matrix from euler angles --- NOT the way you'll be doing it, but easy for this example.

c = (rot*orig')'; %Now, reasign the vertices to be the newly rotated ones.

  del=[x y z]-[0 0 -0.4];
    c = c + repmat(del,[size(orig,1),1]); %We need to add the displacement to ALL the vertices. Hence, repmat lets us duplicate the displacement to match the dimensions of the patch vertex matrix.
     p.Vertices =c;
