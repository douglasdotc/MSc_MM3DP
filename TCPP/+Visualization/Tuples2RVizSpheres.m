function Tuples2RVizSpheres(Vox,varargin)
    disp("Note: displaying array type markers on rviz  requires CPU rendering cuz fuck my life")
    [X, Y, Z]=ndgrid(Vox.spans{1},Vox.spans{2},Vox.spans{3});% expects centerpoints of cox to exist
    N=sum(Vox.N,6);
    N=sum(N,5);
    N=sum(N,4);   
    p=[X(:),Y(:),Z(:)];
    Poses=p(N(:)>0,:);
    N=N(N(:)>0);


    frame = "world";
    id = 0;
    offset_vec = TForm.tform2vec(eye(4));
    scale = [0.3, 0.3, 0.3];
    opacity = 0.5;
    n = size(Poses, 1);    

    while ~isempty(varargin)
        assert(mod(length(varargin), 2) == 0);

        switch lower(varargin{1})
            case "frame"
                frame = varargin{2};
            case "offset_vec"
                offset_vec = varargin{2};
            case "scale"
                scale = varargin{2};
            case "opacity"
                opacity = varargin{2};
            case "id"
                id = varargin{2};
            otherwise
                error(['Unexpected option: ' varargin{1}])
        end

        varargin(1:2) = [];
    end
    
    default_marker = rosmessage("visualization_msgs/Marker");
    default_marker.Action = default_marker.ADD;
    default_marker.Type = default_marker.SPHERELIST;

    default_marker.FrameLocked = true;

    default_marker.Id = id;
    default_marker.Header.FrameId = frame;

    default_marker.Scale.X = scale(1);
    default_marker.Scale.Y = scale(2);
    default_marker.Scale.Z = scale(3);

    default_marker.Pose.Position.X = offset_vec(1);
    default_marker.Pose.Position.Y = offset_vec(2);
    default_marker.Pose.Position.Z = offset_vec(3);
    default_marker.Pose.Orientation.W = offset_vec(4);
    default_marker.Pose.Orientation.X = offset_vec(5);
    default_marker.Pose.Orientation.Y = offset_vec(6);
    default_marker.Pose.Orientation.Z = offset_vec(7);
    
    default_marker.Color.R=0.7;
    default_marker.Color.G=0.5;
    default_marker.Color.B=0.1;
    default_marker.Color.A=1.;

    default_marker.Points =( arrayfun(@(~) rosmessage('geometry_msgs/Point'), zeros(1, n)));
    default_marker.Colors = arrayfun(@(~) rosmessage('std_msgs/ColorRGBA'), zeros(1, n));

    cmap = colormap("parula");
    C = interp1( (1:256)'./256, cmap, N ./ max(N));
    
    for ii = 1:n
        default_marker.Points(ii).X = Poses(ii, 1);
        default_marker.Points(ii).Y = Poses(ii, 2);
        default_marker.Points(ii).Z = Poses(ii, 3);
        default_marker.Colors(ii).R = C(ii, 1);
        default_marker.Colors(ii).G = C(ii, 2);
        default_marker.Colors(ii).B = C(ii, 3);
        default_marker.Colors(ii).A = opacity;
      
        ii/n
    end

    try
        rosinit
    catch
        disp("MATLAB ROS node already initialised")
    end

    pub = rospublisher("matlab/markers", "visualization_msgs/Marker");
    pub.send(default_marker);
    pause(1);
