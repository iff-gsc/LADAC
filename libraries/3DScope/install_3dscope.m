% This file installs the 3DScope S-Functions, according to the 
% running MATLAB version. Copyright 2017 The MathWorks, Inc.

% get MATLAB version
vrs=version;

% look for 3D Scope versions
wm=which('sfunxyz3d.mat','-all');

% make sure we are in the right folder and there are no other 3Dscope files
if length(wm) < 1 
    msg=' Cannot find sfunxyz3d.mat, please run this file from the folder containing sfunxyz3d.mat';
    error(msg);
end

% get the main folder
mp=wm{1}(1:end-14);

% get folder containing sfunxyz
ws=which('sfunxyz','-all');

% find the occurrences of the main folder
% is there is one folder containing sfunxyz which is not the main issue a warning
if str2double(vrs(1:3)) > 6.5
   test=cell2mat(strfind(ws,mp));
   if ~all(test)
      msg=' There is at least another sfunxyz.m or sfunxyz.p file in the path, the installation will go on but it is strongly suggested to delete any other version before using this one';
      warning(msg);
   end
   
end


% load code repository
load sfunxyz3d.mat

% unpack the appropriate p files, depending on the MATLAB version
if str2double(vrs(1:3))<8.4
   fid=fopen('sfunxyz.p','w');fwrite(fid,sfunxyz_p53);fclose(fid);
elseif str2double(vrs(1:3))<9.0
   fid=fopen('sfunxyz.p','w');fwrite(fid,sfunxyz_p86);fclose(fid);
   fid=fopen('sfun3d.p','w');fwrite(fid,sfun3d_p86);fclose(fid);
else
   fid=fopen('sfunxyz.p','w');fwrite(fid,sfunxyz_p86);fclose(fid);
   fid=fopen('sfun3d.m','w');fwrite(fid,sfun3d_m90);fclose(fid);
end

% display feedback message
disp([' 3DScope S-Function(s) for MATLAB ' num2str(vrs(1:3)) ' unpacked']);

% Add target directories and save the updated path
addpath(fullfile(mp,''));
disp(' 3DScope folder added to the path');

if str2double(vrs(1:3)) > 6.5
   result = savepath;
   if result==1
      nl = char(10); %#ok<CHARTEN>
      msg = [' Unable to save updated MATLAB path (<a href="http://www.mathworks.com/support/solutions/en/data/1-9574H9/index.html?solution=1-9574H9">why?</a>)' nl ...
            ' On Windows, exit MATLAB, right-click on the MATLAB icon, select "Run as administrator", and re-run install_3dscope.m' nl ...
            ' On Linux, exit MATLAB, issue a command like this: sudo chmod 777 usr/local/matlab/R2011a/toolbox/local/pathdef.m' nl ...
            ' (depending on where MATLAB is installed), and then re open MATLAB and re-run install_3dscope.m' nl ...
         ];
      warning(msg);
   else
      disp(' Saved updated MATLAB path');
      disp(' ');
   end
end

clear wm mp ws vrs fid sfunxyz_p53 sfunxyz_p86 sfun3d_p86 sfun3d_m90 test result nl msg
