function Tuples2RVizSpheres(Poses, N)

    frame = "world";
    offset_vec = TForm.tform2vec(eye(4));
    scale = [0.03, 0.03, 0.03];
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
            otherwise
                error(['Unexpected option: ' varargin{1}])
        end

        varargin(1:2) = [];
    end

    default_marker = rosmessage("visualization_msgs/Marker");
    default_marker.Action = default_marker.ADD;
    default_marker.Type = default_marker.SPHERELIST;
    default_marker.FrameLocked = true;

    default_marker.Id = "1";
    default_marker.Header.FrameId = frame;

    default_marker.Scale.X = scale(1);
    default_marker.Scale.Y = scale(2);
    default_marker.Scale.Z = scale(3);

    default_marker.Pose.Position.X = pose(1);
    default_marker.Pose.Position.Y = pose(2);
    default_marker.Pose.Position.Z = pose(3);
    default_marker.Pose.Orientation.W = pose(4);
    default_marker.Pose.Orientation.X = pose(5);
    default_marker.Pose.Orientation.Y = pose(6);
    default_marker.Pose.Orientation.Z = pose(7);

    default_marker.Points = arrayfun(@(~) rosmessage('geometry_msgs/Point'), zeros(1, n));
    default_marker.Colors = arrayfun(@(~) rosmessage('std_msgs/ColorRGBA'), zeros(1, n));

    N(ii, 1)
    cmap = colormap("parula");
    C = interp1()

    for ii = 1:n
        default_marker.Points(ii).X = Poses(ii, 1);
        default_marker.Points(ii).Y = Poses(ii, 2);
        default_marker.Points(ii).Z = Poses(ii, 3);
        default_marker.Colors(ii).R =;
        default_marker.Colors(ii).G
        default_marker.Colors(ii).B
        default_marker.Colors(ii).A
    end

    %VOX2RVIZSPHERES Summary of this function goes here
    %   Detailed explanation goes here
    outputArg1 = inputArg1;
    outputArg2 = inputArg2;
end
