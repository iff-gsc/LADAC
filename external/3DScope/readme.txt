3D SCOPE:
--------------------------------------------------------------------------------------------------------------------------------

This block is an enhanced version of the standard XYGraph simulink block.

The standard XYGraph allows plotting the motion of ONE point in the XY plane. 
This block allows plotting the motion of SEVERAL points in the XYZ space. 
This is useful, for example, when comparing two or more trajectories.


USAGE:
--------------------------------------------------------------------------------------------------------------------------------

Make sure you have administrator rights, unzip the 3dscope.zip file into a 
folder of your choice, and run the install_3dscope.m file within MATLAB
(this will install the appropriate S-Functions and add the folder to the 
 MATLAB path).

Run the example models m3dscope_new (R2014b and later only), or 
m3dscope_old (should work on any MATLAB version but I strongly 
encourage you to use the new one if you are running R2014b or later).


NEW VERSION (R2014b and later):
-------------------------------------------------------------------------------------------------------------------------------

The original version of this block was developed in 2004, (based on the 
existing S-Function sfunxy.m for the XY Graph block). This worked fine 
until 2014, but ceased working in 2014b, due to some graphic 
functionalities being retired (and replaced by much better ones).

This new version has been almost completely rewritten to use the new 
features introduced in R2014b, and it is based on a new S-Function,
(sfun3d.m) which is more efficient, accepts more parameters, 
and has a transposed (and more sensible) input ordering, 
(that is the expected input vector now is x1,y1,z1,x2,y2,x2, ... 
 instead of the three vectors x1,x2,... , y1,y2,... , and z1,z2,... )

Have a look at either models m3dscope_new.mdl or lorenz.slx 
to see an usage example of this new block.


LEGACY VERSION (for 2014b and later):
--------------------------------------------------------------------------------------------------------------------------------

The original sfunxyz.m has also been fixed so that it could be used 
in versions 2014b and later. See the model m3dscope.mdl for an example.
Usage is unchanged, so if you have an old block in your model that you
still want to use in newer MATLAB versions, without replacing it with 
the new one, (e.g. because you don't want to rewire the scope inputs)
you can still do it. Just delete the old sfunxyz.m and run install_3dscope.m


MATLAB VERSIONS PRIOR TO R2014b:
--------------------------------------------------------------------------------------------------------------------------------

Since the new scope version (based on sfun3d.m) only works for 2014b and
later, then running install_3dscope on a MATLAB version older than 2014b
will just install the older version of the scope (based on sfunxyz.m).

Note that the examples m3dscope_new.mdl and lorenz.slx will NOT work
with MATLAB versions prior to R2014b.


CONTENTS:
--------------------------------------------------------------------------------------------------------------------------------

readme.txt          This file (please have a look at it).

sfunxyz3d.mat       Mat file containing the appropriate S-Functions
install_3dscope     Installation file

m3dscope_old.mdl    Example using legacy scope based on sfunxyz.m
m3dscope_new.mdl    Example using new scope based on sfun3d.m
lorenz.mdl          Lorenz attractor using new scope based on sfun3d.m

--------------------------------------------------------------------------------------------------------------------------------
Copyright 2017, The MathWorks, Inc.