function [sys, x0, str, ts] = sfun3d(~,~,u,flag,ax,varargin)

%SFUN3D S-function that acts as an X-Y-Z scope using MATLAB plotting functions.
%   This M-file is designed to be used in a Simulink S-function block.
%   It draws a line from the previous input point and the current point.
%
%   NOTE: this is a new version of sfunxyz. It has more natural inputs
%   that is (x1,y1,z1, x2,y2,z2 ... instead of x1,x2, y1,y2, z1,z2 ...)
%   and has the LineStyle and Marker properties as additional parameters,
%   so for versions 2014b and later users should try to use this one
%   instead of the older sfunxyz.
%
%   See also sfunxy, sfunxys.

%   Copyright 2017 The MathWorks, Inc.
%   Based on original work by Andy Grace (May-91), Wes Wang (Apr-93, Aug-93,
%   Dec-93), Craig Santos (Oct-96), and Giampiero Campa (Apr-04, Nov-15,
%   Jan-17, Nov-17).

switch flag
    
    case 0 % [sys,x0,str,ts]=mdlInitializeSizes (ax,varargin) -------------
        % Returns the sizes, initial conditions, and sample times
        % Here is used also to set callbacks, and initialize the figure objects
        
        % version check
        vrs=version;
        if str2double(vrs(1:3))<8.4
            error('This S-Function (sfun3d.m) works only within MATLAB versions 2014b and later. For older versions, please delete any existing version of 3Dscope, (re)install it with this MATLAB version, and use the legacy S-Function ''sfunxyz.m''.');
        end
        
        % build callback strings (to be called when the user performs some actions on the block)
        callbacks={
            'CopyFcn',       'if exist(''sfun3d'',''file''), sfun3d([],[],[],''CopyBlock''); end';
            'DeleteFcn',     'if exist(''sfun3d'',''file''), sfun3d([],[],[],''DeleteBlock''); end';
            'LoadFcn',       'if exist(''sfun3d'',''file''), sfun3d([],[],[],''LoadBlock''); end';
            'NameChangeFcn', 'if exist(''sfun3d'',''file''), sfun3d([],[],[],''NameChange''); end';
            'StartFcn',      '';
            'StopFcn',       '';
            };
        
        % set callbacks as block properties (for these, flag is a string)
        for i=1:length(callbacks)
            if ~strcmp(get_param(gcbh,callbacks{i,1}),callbacks{i,2})
                set_param(gcbh,callbacks{i,1},callbacks{i,2})
            end
        end
        
        % ax was there in sfunxy, for everything else there's varargin
        if length(ax)~=6, error('Axes limits must be defined.'); end
        
        % get number of moving points (i.e. number of lines) to be plotted
        if nargin>6, nmax=fix(varargin{2}); else nmax=1; end
        
        sizes=simsizes; % this initializes size vector to zero
        sizes.NumInputs      = 3*nmax; % input vector size at runtime
        sizes.NumSampleTimes = 1; % fill number of sample times
        
        % get sample time
        if nargin>5, ts=[varargin{1} 0]; else ts=[0.01 0]; end
        
        % return initialization values to simulink as function outputs
        sys=simsizes(sizes);x0=[];str=[];
        
        % get the active figure parameter (toolbar and menubar)
        if nargin>13 && varargin{9}, tb='figure'; else tb='none'; end
        
        % do the figure initialization
        FigHandle=get_param(gcbh,'UserData');
        if isempty(FigHandle) || ~ishandle(FigHandle)
            % the figure doesn't exist, create one
            FigHandle = figure(...
                'Units',          'pixel',...
                'Position',       [100 100 400 300],...
                'Name',           get_param(gcbh,'Name'),...
                'Tag',            'SIMULINK_3DGRAPH_FIGURE',...
                'NumberTitle',    'off',...
                'IntegerHandle',  'off',...
                'Toolbar',        tb,...
                'Menubar',        tb);
        else
            % otherwise clear it
            clf(FigHandle);
        end
        
        % get number of moving points, camera position, and grid switch
        if nargin>7, CPos=varargin{3}; else CPos=[3 2 1]*100; end
        if nargin>8 && varargin{4}, GdSw='On'; else GdSw='Off'; end
        
        % Note: the structure pd contains all the plot data and will be
        % later stored in the figure's userdata!
        
        % create axes
        pd.XYZAxes = axes('Parent',FigHandle);
        cord=get(pd.XYZAxes,'ColorOrder');
        set(pd.XYZAxes,'Visible','on','Xlim', ax(1:2),'Ylim', ax(3:4),'Zlim', ax(5:6),'CameraPosition',CPos,'XGrid',GdSw,'YGrid',GdSw,'ZGrid',GdSw);
        
        % get LineStyle string, Marker string, and max num of line points
        if nargin>9, ls=varargin{5}; else ls='-'; end
        if nargin>10, mk=varargin{6}; else mk='none'; end
        if nargin>11, mx=varargin{7}; else mx=1e5; end
        
        % create a vector of animatedline objects
        pd.XYZLine = [];
        for n=1:nmax
            pd.XYZLine = [pd.XYZLine animatedline('Parent',pd.XYZAxes,'LineStyle',ls,'Marker',mk,'MaximumNumPoints',mx,'Color',cord(1+mod(n-1,size(cord,1)),:))];
        end
        
        % create a vector of line object that will represent the current point position
        if nargin>12 && varargin{8}
            pd.XYZHead = [];
            mrks={'o';'square';'diamond';'v';'+';'*';'x';'^';'>';'<';'pentagram';'hexagram'};
            for n=1:nmax
                pd.XYZHead = [pd.XYZHead line('Parent',pd.XYZAxes,'Marker',mrks{1+mod(n-1,size(mrks,1))},'Color',cord(1+mod(n-1,size(cord,1)),:))];
            end
        end
        
        % create axis labels
        xlabel('X Axis');ylabel('Y Axis');zlabel('Z Axis');
        
        % create plot title
        pd.XYZTitle  = get(pd.XYZAxes,'Title');
        set(pd.XYZTitle,'String','X Y Z Plot');
        
        % Store pd so it can be later retrieved at runtime
        set(FigHandle,'UserData',pd); % store pd in figure's userdata 
        set_param(gcbh,'UserData',FigHandle); % store figure handle in block's UserData
        
    case 2 % sys=mdlUpdate(~,~,u,~) ---------------------------------------
        % Handle discrete state updates, sample time hits, and major time step stuff
        % Here is used only to add another point to the lines
        
        % always return empty, as there are no states
        sys = [];
        
        % Locate the figure window associated with this block.  If it's not a valid
        % handle (it may have been closed by the user), then return.
        FigHandle=get_param(gcbh,'UserData');
        if isempty(FigHandle) || ~ishandle(FigHandle), return, end
        
        % get plot data structure
        pd = get(FigHandle,'UserData');
        
        % add points to each line
        nmax=length(pd.XYZLine);
        for i=1:nmax
            addpoints(pd.XYZLine(i),u(3*(i-1)+1),u(3*(i-1)+2),u(3*(i-1)+3));
        end
        
        % update head position
        if nargin>12 && varargin{8}
            for i=1:nmax
                set(pd.XYZHead(i),'xdata',u(3*(i-1)+1),'ydata',u(3*(i-1)+2),'zdata',u(3*(i-1)+3));
            end
        end
        
        % Note: the following four callbacks were set by "case 0" above
        
    case 'NameChange' % ---------------------------------------------------
        % get the figure associated with this block, if it's valid, change
        % the name of the figure
        FigHandle=get_param(gcbh,'UserData');
        if ishandle(FigHandle)
            set(FigHandle,'Name',get_param(gcbh,'Name'));
        end
        
    case {'CopyBlock', 'LoadBlock'} % -------------------------------------
        % Initialize the block's UserData such that a figure is not associated with the block
        set_param(gcbh,'UserData',-1);
        
    case 'DeleteBlock' % --------------------------------------------------
        % Get the figure handle associated with the block, if it exists, delete it
        FigHandle=get_param(gcbh,'UserData');
        if ishandle(FigHandle)
            delete(FigHandle);
            set_param(gcbh,'UserData',-1);
        end
        
    case {3,9} % ----------------------------------------------------------
        sys=[];
        
    otherwise % -----------------------------------------------------------
        if ischar(flag)
            errmsg=sprintf('Unhandled flag: ''%s''', flag);
        else
            errmsg=sprintf('Unhandled flag: %d', flag);
        end
        error(errmsg);
        
end
% end sfun3d
