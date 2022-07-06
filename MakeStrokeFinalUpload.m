
% parts code from Painterly Rendering with Curved Brush Strokes of Multiple Sizes
%https://github.com/fionazeng3/Painterly-Rendering-with-Curved-Brush-Strokes-of-Multiple-Sizes
%if you don't want draw step by step then comment line 469 drawnow; by
%adding a %

%no soak on paper, all color01 draw only, 4 strokes color refill

close all; clear all; clc;


sourceImage = imread(uigetfile);  % you choose All file type then choose 'reduced8colors.JPG' I uploaded
%sourceImage=imread('reduced8colorswoman.JPG');

R = [8,6,4, 2];

strokeNum=0;   %calculate how many strokes we have

%% parameters 

origin = [109, 185]; % origion of the graph, write as [x y]   note it won't draw out because we put it to origion after draw 
%this origion make the whole drawing within limit (x from -54 to 156, y from 8.83 to 208)

ZOffset=245;  %we can enter the offset value for z here, if enter -100 then all z value will be 100 less


ScaleValue=0.5;  %note it won't draw out because we put it to origion after draw 

ThresholdOfError=100;    %the larger the less exquisite (and less strokes), 2 is exquisite around 60k strokes, 100 is 20k strokes, 200 is 6847 strokes

strokeNum=0;

maxStrokeLength = 1;   %the larger the less exquisite
minStrokeLength = 0.5;   %the larger the less exquisite

fc = 0.1;  %was 0.25 the larger the less exquisite

StrokeSize=10;  %was 10

global RefillStrokes;
RefillStrokes=4; %draw how many strokes then we refill



%% Points
%use [x,y,z,yaw,pitch,roll]

yaw=-176.628586;
pitch=-2.448534;
roll=112.573837;

%we declear globa because we want use them in functions
global ZOfUpGettingColor;
global ZOfDownGettingColor;
global YChangeOfStirColor;
global ZOfUpDrawing;
global ZOfDownDrawing;
% global DistanceBetweenXAxisColors;
% global DistanceBetweenYAxisColors;
% global DistanceBetweenXAxisWithPen;

ZOfUpGettingColor=-79.980942+ZOffset;
ZOfDownGettingColor=-116.275215+ZOffset;
YChangeOfStirColor=13.726929;

ZOfUpDrawing=-79.980942+ZOffset;
ZOfDownDrawing=-113.83+ZOffset;

DistanceBetweenXAxisColors=32; %mm
DistanceBetweenYAxisColors=32; %mm
DistanceBetweenXAxisWithPen=44;
%4.4 mm with the pen

global PaperCenterTip;
global DipWaterTop ;
global DipWaterInside;
global DipWaterMove;
global SoakWaterPaperTop;
global SoakWaterPaperDown;
global SoakWaterPaperRight;
global SoakWaterPaperRightLift;

PaperCenterTip=[109.900528   ,88.328674  ,-30.605270+ZOffset,  yaw ,pitch ,	roll ];
DipWaterTop   =[-205.556641  ,187.694519 ,-30.601479+ZOffset,  yaw ,pitch ,	roll];
DipWaterInside=[ -205.555420 ,187.697021 ,-116.040955+ZOffset, yaw ,pitch ,	roll];
DipWaterMove  =[ -220.532074 ,187.696228 ,-116.040939+ZOffset, yaw ,pitch ,	roll ];
SoakWaterPaperTop=[-205.556000,41.849792 ,-30.737663+ZOffset, yaw ,pitch ,	roll];
SoakWaterPaperDown=[-205.560150,41.852478,-114.775467+ZOffset, yaw ,pitch ,	roll];
SoakWaterPaperRight=[-205.558746,10.343994,-114.774933+ZOffset,yaw ,pitch , roll];
SoakWaterPaperRightLift=[-182.802643,8.211731,-49.687618+ZOffset,yaw ,pitch , roll];


TopOfColor01=[-152.627975 ,177.625671 , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor11=[-152.627975+DistanceBetweenXAxisColors ,177.625671 , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor21=[-152.627975+DistanceBetweenXAxisColors+DistanceBetweenXAxisWithPen ,177.625671 , ZOfUpGettingColor,yaw ,pitch , roll];

TopOfColor02=[-152.627975 ,177.625671- DistanceBetweenYAxisColors , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor12=[-152.627975+DistanceBetweenXAxisColors ,177.625671 - DistanceBetweenYAxisColors, ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor22=[-152.627975+DistanceBetweenXAxisColors+DistanceBetweenXAxisWithPen ,177.625671- DistanceBetweenYAxisColors , ZOfUpGettingColor,yaw ,pitch , roll];

TopOfColor03=[-152.627975 ,177.625671- DistanceBetweenYAxisColors*2 , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor13=[-152.627975+DistanceBetweenXAxisColors ,177.625671 - DistanceBetweenYAxisColors*2, ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor23=[-152.627975+DistanceBetweenXAxisColors+DistanceBetweenXAxisWithPen ,177.625671- DistanceBetweenYAxisColors*2 , ZOfUpGettingColor,yaw ,pitch , roll];

TopOfColor04=[-152.627975 ,177.625671- DistanceBetweenYAxisColors*3 , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor14=[-152.627975+DistanceBetweenXAxisColors ,177.625671 - DistanceBetweenYAxisColors*3, ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor24=[-152.627975+DistanceBetweenXAxisColors+DistanceBetweenXAxisWithPen ,177.625671- DistanceBetweenYAxisColors*3 , ZOfUpGettingColor,yaw ,pitch , roll];

TopOfColor05=[-152.627975 ,177.625671- DistanceBetweenYAxisColors*4 , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor15=[-152.627975+DistanceBetweenXAxisColors ,177.625671 - DistanceBetweenYAxisColors*4, ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor25=[-152.627975+DistanceBetweenXAxisColors+DistanceBetweenXAxisWithPen ,177.625671- DistanceBetweenYAxisColors*4 , ZOfUpGettingColor,yaw ,pitch , roll];

TopOfColor06=[-152.627975 ,177.625671- DistanceBetweenYAxisColors*5 , ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor16=[-152.627975+DistanceBetweenXAxisColors ,177.625671 - DistanceBetweenYAxisColors*5, ZOfUpGettingColor,yaw ,pitch , roll];
TopOfColor26=[-152.627975+DistanceBetweenXAxisColors+DistanceBetweenXAxisWithPen ,177.625671- DistanceBetweenYAxisColors*5 , ZOfUpGettingColor,yaw ,pitch , roll];


%create a matrix store value of in air color positions




%% paint
%At the highest level of this painting algorithm, I first created an empty canvas. Using a for loop,  
% In each iteration, I masked the stroke layer onto the canvas and cumulatively made a complete painting.

[row, column] = size(sourceImage(:,:,1));
canvas = zeros(row, column, 3);
canvas = double(canvas);

for k = 1:length(R)
    referenceImage = sourceImage;


%% paint layer

%search each grid point’s
%neighborhood to find the nearby point with the greatest error
%and paint at this location.


    layer = zeros(size(canvas));
    referenceImage = double(referenceImage);
    L = 0.3 * referenceImage(:,:,1) + 0.59 *referenceImage(:,:,2) + 0.11 * referenceImage(:,:,3);
    [gradientX, gradientY] = gradient(L);
    G = sqrt(gradientX .^2 + gradientY .^2);
    diff = referenceImage - canvas;
    D = sqrt(double(diff(:,:,1)).^2 + double(diff(:,:,2)).^2 + double(diff(:,:,3)).^2);
    grid = R(k);
    T = ThresholdOfError;  %was 50, can set manually
     ygrid = grid:grid:size(referenceImage,1)-grid;
     xgrid = grid:grid:size(referenceImage,2)-grid;
     yorder = ygrid(randperm(length(ygrid)));
     xorder = xgrid(randperm(length(xgrid)));


    for x0 = 1:length(xorder)
        j = xorder(x0)-(grid/2)+1:xorder(x0)+(grid/2);
        for y0 = 1:length(yorder)
             i = yorder(y0)-(grid/2)+1:yorder(y0)+(grid/2);
           % sum the error near (x,y)
            M = D(i,j);
            areaError = sum(sum(M))/(grid*grid);
            if(areaError > T)
                %find the largest error point
                [~, id] = max(M(:));
                [yi, xi] = ind2sub(size(M), id);
                 y1 = yorder(y0) + yi;
                 x1 = xorder(x0) + xi;
 
%% make stroke


   
    S = [x1 y1];
    x = x1;
    y = y1;
    lastDx = 0;
    lastDy = 0;
    strokeRGB = referenceImage(y1, x1, :);

    [row, column] = size(referenceImage(:,:,1));
    for i = 1: maxStrokeLength + 1
        %detect color difference
        isDiff = isColorDiff(referenceImage, canvas, x1, y1, x, y);
        if(i >  minStrokeLength && isDiff == 1)
            break;
        end
        %detect vanishing gradient
        Gx = gradientX(y,x);
        Gy = gradientY(y,x);
        if(G(y,x) == 0)
            break
        end
        %compute a normal direction
        dx = -Gy;
        dy = Gx;
        %if necessary, reverse direction
        if(lastDx * dx + lastDy*dy < 0)
            dx = -dx;
            dy = -dy;
        end
        %filter the stroke direction
       
        dx = fc .* dx + (1-fc) .* lastDx;
        dy = fc .* dy + (1-fc) .* lastDy;
        dx_norm = dx ./ sqrt(dx * dx + dy*dy);
        dy_norm = dy ./sqrt(dx * dx + dy*dy);
        x = x + R(k)*dx_norm;
        y = y + R(k)*dy_norm;
        x = floor(x + R(k)*dx);
        y = floor(y + R(k)*dy);
        xy = [x, y];
        isValid = isValidPoint(x, y, row, column);
        if(isValid == 0)
            break;
        end
        lastDx = dx_norm;
        lastDy = dy_norm;
        S = [S;xy];


    end

              
                strokeRGB = double(strokeRGB);




                if ~isempty(S)
                   % tip = circle(R(k)); 
                   tip = circle(StrokeSize); 
                    tipX = floor(size(tip,2)/2);
                    tipY = floor(size(tip,1)/2);
                    tipR = tip*strokeRGB(1,1,1);
                    tipG = tip*strokeRGB(1,1,2);
                    tipB = tip*strokeRGB(1,1,3);
                    brush = cat(3,tipR,tipG,tipB);
                    [py,px] = size(S);
                    for p = 1: py
                        point = S(p,:);
                        x = point(1,1);
                        y = point(1,2);
                        xMax = size(referenceImage,2) - tipX;
                        xMin = 1 + tipX;
                        yMax = size(referenceImage,1) - tipY;
                        yMin = 1 + tipY;
                        % paints stroke on layer
                        if x >= xMin && x <= xMax && y >= yMin && y <= yMax
                            area = layer(y-tipY:y+tipY,x-tipX:x+tipX,1:3);
                            painted = (brush.*area ~= 0);
                            clean = (painted == 0);
                            layer(y-tipY:y+tipY,x-tipX:x+tipX,1:3) = area + brush.*clean;

                            strokeNum=strokeNum+1;  %every time we loop it we need add one number, will print total strokes later

                            XYMatrixWithColor(strokeNum,1)=x-tipX;   %the first row of XYMatrixWithColor store x-tipX which is begin position's x
                            XYMatrixWithColor(strokeNum,2)=y-tipY;  %the second row of XYMatrixWithColor store y-tipY which is begin position's y
                            XYMatrixWithColor(strokeNum,3)=x+tipX;  %third row end position's x
                            XYMatrixWithColor(strokeNum,4)=y+tipY;  %fourth row end position's y

                            StringStrokeRGB=squeeze(sourceImage(y,x,:));  %we get the origional pixel's color and set it as stroke's color
                           

                            XYMatrixWithColor(strokeNum,5)=StringStrokeRGB(1);
                            XYMatrixWithColor(strokeNum,6)=StringStrokeRGB(2);
                            XYMatrixWithColor(strokeNum,7)=StringStrokeRGB(3);

                             % above we put the color of the points with coordinates (RGB value on the 5th 6th 7th row) because easier to sort

 
                           

                            %plot([x-tipX x+tipX], [y-tipY y+tipY],'color',StringStrokeRGB,'LineWidth',3 )  //display dynamic plotting  
                            %drawnow;
                            %hold on;


                        end
                    end
                end
            end
        end
    end


   % layer = paintLayer(canvas,referenceImage, R(k));
    blank = (layer == 0);
    notLayer = canvas.*blank;

   %%%imshow(uint8(canvas));   %show gradual change process

    canvas = (canvas).*(canvas ~= 0).*(blank) +(canvas ~=0).*(layer ~= 0).*(layer) +(notLayer + layer).*(canvas == 0);
    
end
canvas = uint8(canvas);

oilPaint = canvas;
%%%imshow(oilPaint);

DisplayStroke=['total number of stroke is: '];
disp(DisplayStroke);
disp(strokeNum);

%% Sort by color
%we want sort by color because if we actually draw, it will be more
%efficient to draw with one color then change to another color (no need to wash pen)
%you can also change the color of picture to below standard value using https://onlinepngtools.com/change-png-color
%you can also reduce the color of your image first

% Black=  [0,0,0];
% White=	[255,255,255];
% Red=	[255,0,0];
% Lime=	[0,255,0];
% Blue=	[0,0,255];
% Yellow=	[255,255,0];
% Cyan=	[0,255,255];
% Magenta=[255,0,255];
% Silver=	[192,192,192];
% Gray=	[128,128,128];
% Maroon=	[128,0,0];
% Olive=	[128,128,0];
% Green=	[0,128,0];
% Purple=	[128,0,128];
% Teal=	[0,128,128];
% Navy=	[0,0,128];


%numbering see C:\Users\yushuo\Desktop\Inlead internship\Draw oil painting
%with robot\Vattenfärger 18 st    color numbering.JPG
Color01=  [85, 75, 73];
Color11=	[254, 238, 222];
Color21=	[43, 43, 43];
Color02=	[255, 115, 178];
Color12=	[252, 162, 4];
Color22=	[169, 87, 40];
Color03=	[84, 188, 109];
Color13=    [255, 64, 56];
Color23=	[38, 149, 107];
Color04=	[253, 79, 19];
Color14=	[255, 225, 21];
Color24=	[255, 116, 8];
Color05=	[181, 107, 206];
Color15=	[252, 150, 190];
Color25=	[35, 109, 198];
Color06=	[154, 77, 57];
Color16=	[124, 189, 69];
Color26=	[92, 198, 248];



ColorToleranceR=50;   %if not get enough certain color then change this value
ColorToleranceG=50;
ColorToleranceB=50;

Color01Strokes=SortRGB(Color01,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);  
Color11Strokes=SortRGB(Color11,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color21Strokes=SortRGB(Color21,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color02Strokes=SortRGB(Color02,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color12Strokes=SortRGB(Color12,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color22Strokes=SortRGB(Color22,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color03Strokes=SortRGB(Color03,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color13Strokes=SortRGB(Color13,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color23Strokes=SortRGB(Color23,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color04Strokes=SortRGB(Color04,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color14Strokes=SortRGB(Color14,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color24Strokes=SortRGB(Color24,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color05Strokes=SortRGB(Color05,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color15Strokes=SortRGB(Color15,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color25Strokes=SortRGB(Color25,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color06Strokes=SortRGB(Color06,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color16Strokes=SortRGB(Color16,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);
Color26Strokes=SortRGB(Color26,XYMatrixWithColor,strokeNum,ColorToleranceR,ColorToleranceG,ColorToleranceB);

%% Display number of strokes

Disp1=['total number of stroke for 01 is: '];
disp(Disp1);
disp (size(Color01Strokes,1))  %we count rows to find how many strokes it is

Disp2=['total number of stroke for 11 is: '];
disp(Disp2);
disp (size(Color11Strokes,1))  %we count rows to find how many strokes it is

Disp3=['total number of stroke for 21 is: '];
disp(Disp3);
disp (size(Color21Strokes,1))  %we count rows to find how many strokes it is

Disp4=['total number of stroke for 02 is: '];
disp(Disp4);
disp (size(Color02Strokes,1))  %we count rows to find how many strokes it is

Disp5=['total number of stroke for 12 is: '];
disp(Disp5);
disp (size(Color12Strokes,1))  %we count rows to find how many strokes it is

Disp6=['total number of stroke for 22 is: '];
disp(Disp6);
disp (size(Color22Strokes,1))  %we count rows to find how many strokes it is

Disp7=['total number of stroke for 03 is: '];
disp(Disp7);
disp (size(Color03Strokes,1))  %we count rows to find how many strokes it is

Disp8=['total number of stroke for 13 is: '];
disp(Disp8);
disp (size(Color13Strokes,1))  %we count rows to find how many strokes it is

Disp9=['total number of stroke for 23 is: '];
disp(Disp9);
disp (size(Color23Strokes,1))  %we count rows to find how many strokes it is

Disp10=['total number of stroke for 04 is: '];
disp(Disp10);
disp (size(Color04Strokes,1))  %we count rows to find how many strokes it is

Disp11=['total number of stroke for 14 is: '];
disp(Disp11);
disp (size(Color14Strokes,1))  %we count rows to find how many strokes it is

Disp12=['total number of stroke for 24 is: '];
disp(Disp12);
disp (size(Color24Strokes,1))  %we count rows to find how many strokes it is

Disp13=['total number of stroke for 05 is: '];
disp(Disp13);
disp (size(Color05Strokes,1))  %we count rows to find how many strokes it is

Disp14=['total number of stroke for 15 is: '];
disp(Disp14);
disp (size(Color15Strokes,1))  %we count rows to find how many strokes it is

Disp15=['total number of stroke for 25 is: '];
disp(Disp15);
disp (size(Color25Strokes,1))  %we count rows to find how many strokes it is

Disp16=['total number of stroke for 06 is: '];
disp(Disp16);
disp (size(Color06Strokes,1))  %we count rows to find how many strokes it is

Disp16=['total number of stroke for 16 is: '];
disp(Disp16);
disp (size(Color16Strokes,1))  %we count rows to find how many strokes it is

Disp16=['total number of stroke for 26 is: '];
disp(Disp16);
disp (size(Color26Strokes,1))  %we count rows to find how many strokes it is

%% Sum of all strokes then draw it

SumOfStrokeInColorOrder=[Color01Strokes;Color11Strokes;Color21Strokes;Color02Strokes;Color12Strokes;Color22Strokes;Color03Strokes;Color13Strokes;Color23Strokes;Color04Strokes;Color14Strokes;Color24Strokes;Color05Strokes;Color15Strokes;Color25Strokes;Color06Strokes;Color16Strokes;Color26Strokes];

%the order is BlackStrokes;WhiteStrokes;RedStrokes;LimeStrokes;BlueStrokes;YellowStrokes;CyanStrokes;MagentaStrokes;SilverStrokes;GrayStrokes;MaroonStrokes;OliveStrokes;GreenStrokes;PurpleStrokes;TealStrokes;NavyStrokes

writematrix(SumOfStrokeInColorOrder,'test.txt');   
%we write SumOfStrokeInColorOrder in test.txt, order
% is [x-tipX, y-tipY, x+tipX, y+tipY, GotR, GotG, GotB, UseR, UseG, UseB]
%we have UseR UseG because we have limited paint in hand, don't have all
%color got on picture

%% Draw it

for ooo=1:size(SumOfStrokeInColorOrder,1)
    RGBForEachStroke=[SumOfStrokeInColorOrder(ooo,8) SumOfStrokeInColorOrder(ooo,9) SumOfStrokeInColorOrder(ooo,10)];
      plot([SumOfStrokeInColorOrder(ooo,1) SumOfStrokeInColorOrder(ooo,3)], [SumOfStrokeInColorOrder(ooo,2) SumOfStrokeInColorOrder(ooo,4)],'color',ScaleDown(RGBForEachStroke),'LineWidth',3 )  %display dynamic plotting  
      %here we write as above because we want plot a line, and write as : plot([x1 x2], [y1 y2],'color',xxxxx,'LineWidth',3) 
      
      drawnow;
      hold on;
end

%% Translate to points robot arm can understand (x,y,z,yaw,pitch,roll)

global MatrixForRobotArm;
MatrixForRobotArm=[]; 

for uuu=1:size(SumOfStrokeInColorOrder,1)   
    MatrixForRobotArm=[MatrixForRobotArm;
               SumOfStrokeInColorOrder(uuu,1),SumOfStrokeInColorOrder(uuu,2),ZOfUpDrawing,yaw, pitch,roll;
               SumOfStrokeInColorOrder(uuu,1),SumOfStrokeInColorOrder(uuu,2),ZOfDownDrawing,yaw, pitch,roll;
               SumOfStrokeInColorOrder(uuu,3),SumOfStrokeInColorOrder(uuu,4),ZOfDownDrawing,yaw, pitch,roll;
               SumOfStrokeInColorOrder(uuu,3),SumOfStrokeInColorOrder(uuu,4),ZOfUpDrawing,yaw, pitch,roll];
%we create a new matrix composed by 
% [StartPointX,StartPointY,ZOfUpDrawing,yaw, pitch,roll;
% StartPointX,StartPointY,ZOfDownDrawing,yaw, pitch,roll;
% EndPointX,EndPointY,ZOfDownDrawing,yaw, pitch,roll;
%EndPointX,EndPointY,ZOfUpDrawing,yaw, pitch,roll]
%since we want pen up, pen down, pen up, move to another point, pen down

end

%writematrix(MatrixForRobotArm,'test1.txt'); 

%% Center and Scale
% Find the center point of the graph, then shift to 0,0 then shift to our center point then scale it

SumOfEachColumn=sum(MatrixForRobotArm,1);  %S = sum(A,dim) returns the sum along dimension dim. For example, if A is a matrix, then sum(A,1) is a column vector containing the sum of each column
OldCenterPointX=SumOfEachColumn(1)/size(MatrixForRobotArm,1);  %size(MatrixForRobotArm,1) is number of rows of MatrixForRobotArm 
OldCenterPointY=SumOfEachColumn(2)/size(MatrixForRobotArm,1);

for jjj = 1:size(MatrixForRobotArm,1)
      MatrixForRobotArm(jjj,1)=(MatrixForRobotArm(jjj,1)-OldCenterPointX+origin(1))*ScaleValue;   
      MatrixForRobotArm(jjj,2)=(MatrixForRobotArm(jjj,2)-OldCenterPointY+origin(2))*ScaleValue;  
end

figure('Name','Centered Scaled points'); %create a new figure window to draw, see new points
plot(MatrixForRobotArm(:,1),MatrixForRobotArm(:,2)); 
%if we want print 3d graph use   plot3(MatrixForRobotArm(:,1),MatrixForRobotArm(:,2),MatrixForRobotArm(:,3)); 

%% Count from which line does different color start

%count to that line we have that color, in cumulative amount (so that we know which row to which row is what color)

NumberOfLineForColor01=size(Color01Strokes,1)*4;
NumberOfLineForColor11=NumberOfLineForColor01+size(Color11Strokes,1)*4;
NumberOfLineForColor21=NumberOfLineForColor11+size(Color21Strokes,1)*4;

NumberOfLineForColor02=NumberOfLineForColor21+size(Color02Strokes,1)*4;
NumberOfLineForColor12=NumberOfLineForColor02+size(Color12Strokes,1)*4;
NumberOfLineForColor22=NumberOfLineForColor12+size(Color22Strokes,1)*4;

NumberOfLineForColor03=NumberOfLineForColor22+size(Color03Strokes,1)*4;
NumberOfLineForColor13=NumberOfLineForColor03+size(Color13Strokes,1)*4;
NumberOfLineForColor23=NumberOfLineForColor13+size(Color23Strokes,1)*4;

NumberOfLineForColor04=NumberOfLineForColor23+size(Color04Strokes,1)*4;
NumberOfLineForColor14=NumberOfLineForColor04+size(Color14Strokes,1)*4;
NumberOfLineForColor24=NumberOfLineForColor14+size(Color24Strokes,1)*4;

NumberOfLineForColor05=NumberOfLineForColor24+size(Color05Strokes,1)*4;
NumberOfLineForColor15=NumberOfLineForColor05+size(Color15Strokes,1)*4;
NumberOfLineForColor25=NumberOfLineForColor15+size(Color25Strokes,1)*4;

NumberOfLineForColor06=NumberOfLineForColor25+size(Color06Strokes,1)*4;
NumberOfLineForColor16=NumberOfLineForColor06+size(Color16Strokes,1)*4;
NumberOfLineForColor26=NumberOfLineForColor16+size(Color26Strokes,1)*4;

%tested size(MatrixForRobotArm,1)==NumberOfLineForColor26



global SumOfGettingColor;
SumOfGettingColor=[];

LoopAndAppend(NumberOfLineForColor01,1, TopOfColor01);  %it loop over old matrix, append depending on situation
LoopAndAppend(NumberOfLineForColor11,NumberOfLineForColor01, TopOfColor11);
LoopAndAppend(NumberOfLineForColor21,NumberOfLineForColor11, TopOfColor21);
LoopAndAppend(NumberOfLineForColor02,NumberOfLineForColor21, TopOfColor02); 
LoopAndAppend(NumberOfLineForColor12,NumberOfLineForColor02, TopOfColor12);
LoopAndAppend(NumberOfLineForColor22,NumberOfLineForColor12, TopOfColor22);
LoopAndAppend(NumberOfLineForColor03,NumberOfLineForColor22, TopOfColor03); 
LoopAndAppend(NumberOfLineForColor13,NumberOfLineForColor03, TopOfColor13);
LoopAndAppend(NumberOfLineForColor23,NumberOfLineForColor13, TopOfColor23);
LoopAndAppend(NumberOfLineForColor04,NumberOfLineForColor23, TopOfColor04); 
LoopAndAppend(NumberOfLineForColor14,NumberOfLineForColor04, TopOfColor14);
LoopAndAppend(NumberOfLineForColor24,NumberOfLineForColor14, TopOfColor24);
LoopAndAppend(NumberOfLineForColor05,NumberOfLineForColor24, TopOfColor05); 
LoopAndAppend(NumberOfLineForColor15,NumberOfLineForColor05, TopOfColor15);
LoopAndAppend(NumberOfLineForColor25,NumberOfLineForColor15, TopOfColor25);
LoopAndAppend(NumberOfLineForColor06,NumberOfLineForColor25, TopOfColor06); 
LoopAndAppend(NumberOfLineForColor16,NumberOfLineForColor06, TopOfColor16);
LoopAndAppend(NumberOfLineForColor26,NumberOfLineForColor16, TopOfColor26);

%if we use all of them, with 20k strokes, we will get 200k lines (200k
%points), we can just try one color first

writematrix(SumOfGettingColor,'test1.txt'); 

%% function loop and append, determine where needs to add color

function ttt=LoopAndAppend(NumberOfLineForColorxx, PriviousNumOfLine, TopOfColorxx)
global RefillStrokes;
global SumOfGettingColor;
global MatrixForRobotArm;


for vvvv=PriviousNumOfLine:NumberOfLineForColorxx
    SumOfGettingColor=[SumOfGettingColor;MatrixForRobotArm(vvvv,:)]; %append every row to new matrix

    if rem ( vvvv , RefillStrokes*4 )==0  
        %if reminder is 0 means the remider of vvvv divided by RefillStrokes*4 is 0.
        %use 4 because each stroke take 4 rows
        SumOfGettingColor=[SumOfGettingColor;AddColor(TopOfColorxx)]; %append AddColor function which is a matrix
    end

end
 
end

 

%% add color function

function bbbbb=AddColor(TopOfTheColor)
SumOfGettingColor=[];
global ZOfUpGettingColor;
global ZOfDownGettingColor;
global YChangeOfStirColor;
global PaperCenterTip;
global DipWaterTop ;
global DipWaterInside;
global DipWaterMove;
global SoakWaterPaperTop;
global SoakWaterPaperDown;
global SoakWaterPaperRight;
global SoakWaterPaperRightLift;

        DownOfTheColor=[TopOfTheColor(1),TopOfTheColor(2),ZOfDownGettingColor,TopOfTheColor(4),TopOfTheColor(5),TopOfTheColor(6)];  %down get color
        StirOfTheColor=[TopOfTheColor(1),TopOfTheColor(2)+YChangeOfStirColor,ZOfDownGettingColor,TopOfTheColor(4),TopOfTheColor(5),TopOfTheColor(6)];  %stir color
        %SumOfGettingColor=[SumOfGettingColor;DipWaterTop;DipWaterInside;DipWaterMove;DipWaterInside;DipWaterMove;DipWaterInside;DipWaterTop;TopOfTheColor;DownOfTheColor;StirOfTheColor;DownOfTheColor;StirOfTheColor;DownOfTheColor;TopOfTheColor];
        %above has no soak water on paper, if want then use the below line
        %instead

        SumOfGettingColor=[SumOfGettingColor;DipWaterTop;DipWaterInside;DipWaterMove;DipWaterInside;DipWaterMove;DipWaterInside;DipWaterTop;SoakWaterPaperTop;SoakWaterPaperDown;SoakWaterPaperRight;SoakWaterPaperRightLift;TopOfTheColor;DownOfTheColor;StirOfTheColor;DownOfTheColor;StirOfTheColor;DownOfTheColor;TopOfTheColor];
        
        %we first dip water, then wipe water on paper, then move to the
        %color, get down to get the color. we can change appended array to
        %change movements
        %if is oil painting then no need to dip water unless switch color,
        %become SumOfGettingColor=[SumOfGettingColor;TopOfTheColor;DownOfTheColor;StirOfTheColor;DownOfTheColor;StirOfTheColor;DownOfTheColor;TopOfTheColor];
bbbbb=SumOfGettingColor;
end

%% Sort RGB function
%we want sort RGB on the picture to one of the standard color we gave
%earlier (since we have limited paint)
function vvv = SortRGB(RGB,XYMatrixWithColorr,strokeNumm,ColorToleranceRR,ColorToleranceGG,ColorToleranceBB)   %we need stroke numm here because it is within the function
PointsOfTheColor=[];   
 for iii=1: strokeNumm 

     % see if R match first including tolerance, we use this because is easier to see
     if (RGB(1)-ColorToleranceRR)<XYMatrixWithColorr(iii,5) && XYMatrixWithColorr(iii,5) <(RGB(1)+ColorToleranceRR)
       RMatch=1;
     else
       RMatch=0;  
     end

     if (RGB(2)-ColorToleranceGG)<XYMatrixWithColorr(iii,6) && XYMatrixWithColorr(iii,6) <(RGB(2)+ColorToleranceGG)
       GMatch=1;
     else
       GMatch=0;  
     end

     if (RGB(3)-ColorToleranceBB)<XYMatrixWithColorr(iii,7) && XYMatrixWithColorr(iii,7) <(RGB(3)+ColorToleranceBB)
       BMatch=1;
     else
       BMatch=0;  
     end


     if RMatch==1 && GMatch==1 && BMatch==1

       PointsOfTheColor=[PointsOfTheColor;XYMatrixWithColorr(iii,1) XYMatrixWithColorr(iii,2) XYMatrixWithColorr(iii,3) XYMatrixWithColorr(iii,4) XYMatrixWithColorr(iii,5) XYMatrixWithColorr(iii,6) XYMatrixWithColorr(iii,7) RGB(1) RGB(2) RGB(3)];
       
     % XYMatrixWithColorr(iii,:)=0;  %after we assign the color we replace that row with 0 means the stroke is taken and won't have multiple color

     %Append rows at the end of Matrix each time condition is met, then we
     %append the color we are going to use as well
     %so the total matrix is [x-tipX, y-tipY, x+tipX, y+tipY, GotR, GotG, GotB, UseR, UseG, UseB]
     %GotR is the R value we got by squeeze the origional image, useR is
     %the actual R we will use to paint

     end
 end
 vvv=PointsOfTheColor;
end


%% Scale RGB function
%we need it because when draw it accept value between 0 to 1, and what we
%got is 0 to255
function ScaledRGB = ScaleDown(RGBname)
RGBname(1)=RGBname(1)/255;
RGBname(2)=RGBname(2)/255;
RGBname(3)=RGBname(3)/255;
ScaledRGB=RGBname;
end

%% isValidPoint

function isValid = isValidPoint(x, y, row, column)
if (x >= 1 && x <= column && y >= 1 && y <= row)
    isValid = 1;
else 
    isValid = 0;
end
end

%% is color diff

function isDiff = isColorDiff(referenceImage, canvas, x0, y0, x, y)
    %|refImage.color(x,y)-canvas.color(x,y)|<|refImage.color(x,y)-strokeColor|)
    left = norm(double(referenceImage(y,x) - canvas(y,x)));
    right = norm(double(referenceImage(y,x) -referenceImage(y0,x0)));


    if(left < right)
        isDiff = 1;
    else 
        isDiff = 0;
    end
end

%% creates circular brush element for a given radius

function c = circle(R)
% creates circular brush element for a given radius
if R < 3 % keeps non-zero brush element circular and not square
    R = R + 1;
end
c = zeros(R);
for x = R:-1:1 % for one quadrant of circle
    y = (R^2 - (x-1)^2)^(1/2);
    y = floor(y);
    c(y:-1:1,x) = ones(y,1);
end
% forms circle out of quadrant
c = [c(end:-1:2,end:-1:2), c(end:-1:2,:); c(:,end:-1:2), c];
end







