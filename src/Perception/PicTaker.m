function PicTaker(str, numofpics)
%PicTaker takes screnshots to produce picture for image training
 
mkdir(str + "s");
cd(str + "s");
t = timer('TimerFcn', 'stat=false; disp(''Timer!'')',... 
                'StartDelay',10);
start(t)
%wait a few seconds to start
pause(5)
count = 0;
while(count < numofpics)
  disp(count)
    %takes screnshot to produce picture for image training
    robot = java.awt.Robot();
    pos = [40 40 2400 1400]; % [left top width height]
    rect = java.awt.Rectangle(pos(1),pos(2),pos(3),pos(4));
    cap = robot.createScreenCapture(rect);
    javax.imageio.ImageIO.write(cap ,"png",java.io.File(str + count + ".png"));
    count = count + 1;
end
cd ..;
end