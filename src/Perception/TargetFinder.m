classdef TargetFinder
    %Finds the target in the image
    
    properties
        net
        robot
    end
    
    methods
        function obj = AutoPlayer(inputnet)
            %Construct an instance of this class
            %   inputnet is a trained net 
            obj.net = inputnet;
            obj.robot = java.awt.Robot();
        end
        function report = canSeeTarget(obj)
            report = obj.scout();
        end
        function outputArg = scout(obj)
            %scouts what the robot is currently looking at 
            pos = [40 40 2400 1400]; % [left top width height]
            rect = java.awt.Rectangle(pos(1),pos(2),pos(3),pos(4));
            cap = obj.robot.createScreenCapture(rect);
            rgb = typecast(cap.getRGB(0,0,cap.getWidth,cap.getHeight,[],0,cap.getWidth),'uint8');
            imgData = zeros(cap.getHeight,cap.getWidth,3,'uint8');
            imgData(:,:,1) = reshape(rgb(3:4:end),cap.getWidth,[])';
            imgData(:,:,2) = reshape(rgb(2:4:end),cap.getWidth,[])';
            imgData(:,:,3) = reshape(rgb(1:4:end),cap.getWidth,[])';
            J = imresize(imgData, [227, 227]);
            output = classify(obj.net, J);
            disp(output)
            outputArg = output;
        end
        
    end
end

