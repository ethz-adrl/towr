function MainFig = mouse_figure(MainFig)
%MOUSE_FIGURE           mouse-friendly figure
%
% MOUSE_FIGURE() creates a figure (or modifies an existing one) that allows
% zooming with the scroll wheel and panning with mouse clicks, *without*
% first selecting the ZOOM or PAN tools from the toolbar. Moreover, zooming
% occurs to and from the point the mouse currently hovers over, instead of
% to and from the less intuitive "CameraPosition". 
%
%         Scroll: zoom in/out
%     Left click: pan
%   Double click: reset view to default view
%    Right click: set new default view
%
% LIMITATIONS: This function (re-)efines several functions in the figure 
% (WindowScrollWheelFcn, WindowButtonDownFcn, WindowButtonUpFcn and 
% WindowButtonMotionFcn), so if you have any of these functions already
% defined they will get overwritten. Also, MOUSE_FIGURE() only works 
% properly for 2-D plots. As such, it should only be used for simple, 
% first-order plots intended for "quick-n-dirty" data exploration.
%
% EXAMPLE:
%
%   mouse_figure;
%   x = linspace(-1, 1, 10000);
%   y = sin(1./x);
%   plot(x, y) 
%   
% See also figure, axes, zoom, pan.


% Please report bugs and inquiries to: 
%
% Name       : Rody P.S. Oldenhuis
% E-mail     : oldenhuis@gmail.com    (personal)
%              oldenhuis@luxspace.lu  (professional)
% Affiliation: LuxSpace s�rl
% Licence    : GPL + anything implied by placing it on the FEX


% If you find this work useful, please consider a donation:
% https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=6G3S5UYM7HJ3N

    
    % initialize
    status = '';  previous_point = [];
    
    % initialize axes
    if (nargin == 0) || ~ishandle(MainFig)
        MainFig = figure;  axs = gca;
    else
        axs = get(MainFig, 'currentaxes');
    end
    
    % only works properly for 2D plots
    if ~is2D(axs) % is2D might disappear in a future release...
        error('mouse_figure:plot3D_not_supported', ...
              'MOUSE_FIGURE() only works for 2-D plots.');
    end
        
    % get original limits
    original_xlim = get(axs, 'xlim');
    original_ylim = get(axs, 'ylim');
    
    % define zooming with scrollwheel, and panning with mouseclicks
    set(MainFig, ...
        'WindowScrollWheelFcn' , @scroll_zoom,...
        'WindowButtonDownFcn'  , @pan_click,...
        'WindowButtonUpFcn'    , @pan_release,...
        'WindowButtonMotionFcn', @pan_motion);    
    
    % zoom in to the current point with the mouse wheel
    function scroll_zoom(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % get the amount of scolls
        scrolls = varargin{2}.VerticalScrollCount;
        % get the axes' x- and y-limits
        xlim = get(axs, 'xlim');  ylim = get(axs, 'ylim');
        % get the current camera position, and save the [z]-value
        cam_pos_Z = get(axs, 'cameraposition');  cam_pos_Z = cam_pos_Z(3);
        % get the current point
        old_position = get(axs, 'CurrentPoint'); old_position(1,3) = cam_pos_Z;
        % calculate zoom factor
        zoomfactor = 1 - scrolls/5;
        % adjust camera position
        set(axs, 'cameratarget', [old_position(1, 1:2), 0],...
            'cameraposition', old_position(1, 1:3));
        % adjust the camera view angle (equal to zooming in)
        camzoom(zoomfactor);
        % zooming with the camera has the side-effect of
        % NOT adjusting the axes limits. We have to correct for this:
        x_lim1 = (old_position(1,1) - min(xlim))/zoomfactor;
        x_lim2 = (max(xlim) - old_position(1,1))/zoomfactor;
        xlim   = [old_position(1,1) - x_lim1, old_position(1,1) + x_lim2];
        y_lim1 = (old_position(1,2) - min(ylim))/zoomfactor;
        y_lim2 = (max(ylim) - old_position(1,2))/zoomfactor;
        ylim   = [old_position(1,2) - y_lim1, old_position(1,2) + y_lim2];
        set(axs, 'xlim', xlim), set(axs, 'ylim', ylim)
        % set new camera position
        new_position = get(axs, 'CurrentPoint');
        old_camera_target =  get(axs, 'CameraTarget');
        old_camera_target(3) = cam_pos_Z;
        new_camera_position = old_camera_target - ...
            (new_position(1,1:3) - old_camera_target(1,1:3));
        % adjust camera target and position
        set(axs, 'cameraposition', new_camera_position(1, 1:3),...
            'cameratarget', [new_camera_position(1, 1:2), 0]);
        % we also have to re-set the axes to stretch-to-fill mode
        set(axs, 'cameraviewanglemode', 'auto',...
            'camerapositionmode', 'auto',...
            'cameratargetmode', 'auto');
    end % scroll_zoom
    
    % pan upon mouse click
    function pan_click(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % perform appropriate action
        switch lower(get(MainFig, 'selectiontype'))            
            % start panning on left click
            case 'normal' 
                status = 'down';
                previous_point = get(axs, 'CurrentPoint');              
            % reset view on double click
            case 'open' % double click (left or right)
                set(axs, 'Xlim', original_xlim,...
                         'Ylim', original_ylim);  
            % right click - set new reset state
            case 'alt'
                original_xlim = get(axs, 'xlim');
                original_ylim = get(axs, 'ylim');
        end
    end
    
    % release mouse button
    function pan_release(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % just reset status
        status = '';
    end
    
    % move the mouse (with button clicked)
    function pan_motion(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % return if there isn't a previous point
        if isempty(previous_point), return, end  
        % return if mouse hasn't been clicked
        if isempty(status), return, end  
        % get current location (in pixels)
        current_point = get(axs, 'CurrentPoint');
        % get current XY-limits
        xlim = get(axs, 'xlim');  ylim = get(axs, 'ylim');     
        % find change in position
        delta_points = current_point - previous_point;  
        % adjust limits
        new_xlim = xlim - delta_points(1); 
        new_ylim = ylim - delta_points(3); 
        % set new limits
        set(axs, 'Xlim', new_xlim); set(axs, 'Ylim', new_ylim);           
        % save new position
        previous_point = get(axs, 'CurrentPoint');
    end 
    
end

