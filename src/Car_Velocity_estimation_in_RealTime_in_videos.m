/*
 * The MIT License
 *
 * Copyright 2017 amr.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
* Note : this Code based on matlab 2013 a example on similar task
*to use please refer to amrabdellatief1@gmail.com  Amr A. El Latief
*feedback : amrabdellatief1@gmail.com
*/

%Read Video File 

hbfr = video.MultimediaFileReader( 'Filename', 'D:\datavideos\videonetgood5.avi');
 
widthofopject = mmreader('D:\datavideos\videonetgood5.avi');
widthofopject.width
 
 
 
%hbfr = video.MultimediaFileReader( 'Filename', 'viptraffic.avi');
%widthofopject = mmreader('viptraffic.avi');
%widthofopject.width
 
 


%Take input from user


prompt = 'What is the angle of camera?';
camera = input(prompt)
 
prompt = 'What is the Distance from the camera ?';
Distance = input(prompt)
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%perform speed calculations : 
 
%Speed initial Calculations :

 
 
viewdistance =  (Distance*sind(camera))/ (sind(90-(camera/2)));
 
viewdistance
 
 
%%the distances
 
 
 
cmperpixel=viewdistance/widthofopject.width
 
meterperpixel =cmperpixel/100
 
kmperpixel = meterperpixel/1000
 
 
%Video initial Operations :




%aplly optical flow  
 
hof = video.OpticalFlow( ...
    'OutputValue', 'Horizontal and vertical components in complex form', ...
    'ReferenceFrameDelay', 3);
 
%Create and configure two System objects, one for calculating the overall mean of the velocities and the other for calculating the running mean of the velocities.
 
hmean1 = video.Mean;
hmean2 = video.Mean('RunningMean', true);
 
 
%Create a 2-D median filtering System object to filter out slight speckle noise introduced due to segmentation.
 
hmedianfilt = video.MedianFilter2D;
 
%Create a morphological closing System object used to remove gaps within the blobs.
 
hclose = video.MorphologicalClose('Neighborhood', strel('line',10,45));
 
 
%Create a blob analysis System object to segment cars in the video.
 
hblob = video.BlobAnalysis( ...
                    'CentroidOutputPort', true, ...
                    'AreaOutputPort', true, ...
                    'BoundingBoxOutputPort', true, ...
                    'OutputDataType', 'double', ...
                    'NumBlobsOutputPort',  false, ...
                    'MinimumBlobAreaSource', 'Property', ...
                    'MinimumBlobArea', 300, ...
                    'MaximumBlobAreaSource', 'Property', ...
                    'MaximumBlobArea', 2000, ...
                    'FillValues', -1, ...
                    'MaximumCount', 20);
 
%Create a morphological erosion System object to thin out portions of the road and objects other than cars which are also produced as a result of segmentation.
 
herode = video.MorphologicalErode('Neighborhood', strel('square',2));
 
%Create and configure two System objects that insert shapes, one for drawing the bounding box around the cars and the other for drawing the motion vector lines.
 
hshapeins1 = video.ShapeInserter( ...
            'BorderColor', 'Custom', ...
            'CustomBorderColor', [0 1 0]);
hshapeins2 = video.ShapeInserter( ...
            'Shape','Lines', ...
            'BorderColor', 'Custom', ...
            'CustomBorderColor', [255 255 0]);
 
%Create and configure a System object to write the number of cars being tracked.
 
htextins = video.TextInserter( 'Text', '%4d','Location',  [0 0],'Color', [0 1 0], 'Font', 'Arial','FontSize', 14);
    
    
    
     H = video.TextInserter(...
        'Text', '%4d', ...
        'Location','Input port', ...
        'Color', [0 1 0], ...
        'Font', 'Arial', ...
        'FontSize', 16);    
 textInserter = video.TextInserter('%d','LocationSource','Input port','Color',[0,255, 0],'FontSize',16);
 
 
 
%'Input port'
 
 
%Video windwos: 
 
%Create System objects to display the original video, motion vector video, the thresholded video and the results.
 
hVideo1 = video.VideoPlayer('WindowCaption', 'Original Video');
hVideo1.WindowPosition(1) = round(0.4*hVideo1.WindowPosition(1)) ;
hVideo1.WindowPosition(2) = round(1.5*(hVideo1.WindowPosition(2))) ;
hVideo1.WindowPosition([4 3]) = [(widthofopject.height+50) widthofopject.width];
 
hVideo2 = video.VideoPlayer('WindowCaption', 'Motion Vector');
hVideo2.WindowPosition(1) = hVideo1.WindowPosition(1) + widthofopject.width +50;
hVideo2.WindowPosition(2) =round(1.5* hVideo2.WindowPosition(2));
hVideo2.WindowPosition([4 3]) = [(widthofopject.height+50) widthofopject.width];
 
hVideo3 = video.VideoPlayer('WindowCaption', 'Thresholded Video');
hVideo3.WindowPosition(1) = hVideo2.WindowPosition(1) +  widthofopject.width +50;
hVideo3.WindowPosition(2) = round(1.5*(hVideo3.WindowPosition(2))) ;
hVideo3.WindowPosition([4 3]) = [(widthofopject.height+50) widthofopject.width];
 
hVideo4 = video.VideoPlayer('WindowCaption', 'Results');
hVideo4.WindowPosition(1) = hVideo1.WindowPosition(1);
hVideo4.WindowPosition(2) = round(0.3*(hVideo4.WindowPosition(2))) ;
hVideo4.WindowPosition([4 3]) = [(widthofopject.height+50) widthofopject.width];
 
 
 
hVideo5 = video.VideoPlayer('WindowCaption', 'Real velocity');
hVideo5.WindowPosition(1) = hVideo1.WindowPosition(1);
hVideo5.WindowPosition(2) = round(0.3*(hVideo5.WindowPosition(2))) ;
hVideo5.WindowPosition([4 3]) = [(widthofopject.height+50) widthofopject.width];
 

%Initialization of some Variables:
 
% Initialize some variables used in plotting motion vectors.
MotionVecGain = 20;
line_row =  22;
borderOffset   = 5;
decimFactorRow = 5;
decimFactorCol = 5;
firstTime = true;
noofframes =int32(0);

%Main Loop :

%Stream processing loop
 
%Create the processing loop to track the cars in the input video. This loop uses the System objects previously instantiated.
 
%The loop is stopped when you reach the end of the input file, which is detected by the BinaryFileReader System object.
 
while ~isDone(hbfr)
   % [y, cb, cr] = step(hbfr);      % Read input video frame
    
   
   image = step(hbfr); % Read input video frame
   
  
     noofframes = int32(noofframes+1);                          % increment frame counter 
 
 I = rgb2gray(image);
 
 %   I = step(my_rgb_frame, image);        % Convert color image to intensity
    of = step(hof, I);             % Estimate optical flow
 
    % Thresholding and Region Filtering.
    y1 = of .* conj(of);
    % Compute the velocity threshold from the matrix of complex velocities.
    vel_th = 0.5 * step(hmean2, step(hmean1, y1));
 
    % Threshold the image and then filter it to remove fine speckle noise.
    filteredout = step(hmedianfilt, y1 >= vel_th);
 
    % Perform erosion operation to thin-out the parts of the road followed
    % by the closing operation to remove gaps in the blobs.
    th_image = step(hclose, step(herode, filteredout));
 
    % Regional Filtering.
 
    % Estimate the area and bounding box of the blobs in the thresholded
    % image.
    
 %Store previous Values : 
      
    % adding store the centroid of previous loop 
    
    
    %for five opjects 
    
    if exist('Centroid')
    
    previousCentroid1 = Centroid(1);    previousCentroid2 = Centroid(2);
    
    
     previousCentroid3 = Centroid(3);    previousCentroid4 = Centroid(4);
    
     previousCentroid5 = Centroid(5);    previousCentroid6 = Centroid(6);
    
    previousCentroid7 = Centroid(8);    previousCentroid8 = Centroid(8);
     
    else 
        
        previousCentroid1 =0;
        previousCentroid2 =0;
        previousCentroid3 = 0;
        previousCentroid4 =0;
        previousCentroid5 =0;
        previousCentroid6 =0;
        previousCentroid7 =0;
        previousCentroid8 =0;
    end;
      
  
%perform Blop Analysis:     
    
    [area, Centroid, bbox] = step(hblob, th_image);
       
    % Select those boxes which are in our ROI.
    Idx = bbox(1,:) > line_row;
 
    % The next lines of code exclude other objects (like parts of the road)
    % which are also segmented as blobs and select only cars. When the
    % ratio between the area of the blob and the area of the bounding box
    % is above 0.4 (40%), it is considered as a car and hence the bounding
    % box for that object is used. Otherwise the bounding box is removed.
    ratio = zeros(1, length(Idx));
    ratio(Idx) = single(area(1,Idx))./single(bbox(3,Idx).*bbox(4,Idx));
    ratiob = ratio > 0.4;
    count = int32(sum(ratiob));    % Number of cars
    bbox(:, ~ratiob) = int32(-1);
 
    
    % Draw bounding rectangles around the tracked cars.
    
    %y2 = step(hshapeins1, image,bbox);
   
  
% calculate Distance and speed for opjects:  
   
    % for  opject 1
    
    speed1x =  Centroid(1) - previousCentroid1;             % in pixel / frame
   
   speed1y =  Centroid(2) - previousCentroid2;              % pixel / frame
   
   
   
        speedfp1 = uint32(sqrt(speed1x^2 + speed1y^2))   
   
        speedfp1realmeterpersec = uint32((sqrt(speed1x^2 + speed1y^2))*meterperpixel*(24))   
   
        
   
        speedfp1realmeterperhour = uint32((sqrt(speed1x^2 + speed1y^2))*meterperpixel*24*3600)   
      
   
        speedfp1realkmperhour = uint32((sqrt(speed1x^2 + speed1y^2))*kmperpixel*24*3600)   
   
   
        %for opject 2
        
    speed2x =  Centroid(3) - previousCentroid3;             % in pixel / frame
   
   speed2y =  Centroid(4) - previousCentroid4;              % pixel / frame
   
   
     
        speedfp2 = uint32(sqrt(speed2x^2 + speed2y^2));   
   
        speedfp2realmeterpersec = uint32((sqrt(speed2x^2 + speed2y^2))*meterperpixel*24);   
   
   
        speedfp2realmeterperhour = uint32((sqrt(speed2x^2 + speed2y^2))*meterperpixel*24*3600);   
   
        
   
        speedfp2realkmperhour = uint32((sqrt(speed2x^2 + speed2y^2))*kmperpixel*24*3600);   
   
        
        
      speed3x =  Centroid(5) - previousCentroid5;             % in pixel / frame
   
   speed3y =  Centroid(6) - previousCentroid6;              % pixel / frame
   
   
     
        speedfp3 =uint32(sqrt(speed3x^2 + speed3y^2));   
 
                speedfp3realmeterpersec = uint32((sqrt(speed3x^2 + speed3y^2))*meterperpixel*24);   
 
             speedfp3realmeterperhour = uint32((sqrt(speed3x^2 + speed3y^2))*meterperpixel*24*3600);   
        
          speedfp3realkmperhour = uint32((sqrt(speed3x^2 + speed3y^2))*meterperpixel*24*3600);   
        
        
        
        speed4x =  Centroid(7) - previousCentroid7;             % in pixel / frame
   
   speed4y =  Centroid(8) - previousCentroid8;              % pixel / frame
   
   
   %b = ' tracked is';
   
     
   %c = [b,' ',num2str(count)]
   
     
        speedfp4 = uint32(sqrt(speed4x^2 + speed4y^2));   
      
        
        
        speedfp4realmeterpersec = uint32((sqrt(speed4x^2 + speed4y^2))*meterperpixel*24);   
        
        
        speedfp4realmeterperhour = uint32((sqrt(speed4x^2 + speed4y^2))*meterperpixel*24*3600);   
        
        
        speedfp4realkmperhour = uint32((sqrt(speed4x^2 + speed4y^2))*kmperpixel*24*3600);
        
        %  centroidmat = [Centroid(1) Centroid(3);Centroid(2) Centroid(4) ];
   
    centroidmat = [(Centroid(1)) (Centroid(3)) (Centroid(5)) (Centroid(7));(Centroid(2)) (Centroid(4)) (Centroid(6)) (Centroid(8))];
   
     
    % S = {'Hello'; 'Goodbye'};
    
     
    % velocity frame per pixel
    
    velocitiesppf =int32([speedfp1 speedfp2 speedfp3 speedfp4]);
    
    
    % velocity meter per sec
    
    %velocitiesppfmeterpersec =int32([speedfp1realmeterpersec speedfp2realmeterpersec speedfp3realmeterpersec speedfp4realmeterpersec])
    
   
   % meter per hour
   
 
    % velocitiesppfmeterperhour =int32([speedfp1realmeterperhour speedfp2realmeterperhour speedfp3realmeterperhour speedfp4realmeterperhour]);
        
     velocitiesppfkmperhor =int32([speedfp1realkmperhour speedfp2realkmperhour speedfp3realkmperhour speedfp4realkmperhour])
    
     
  y2 = step(textInserter, image,velocitiesppf,uint32(centroidmat));
   
 
    % Display the number of cars tracked and a white line showing ROI.
    y2(22:23,:,:) = 1;             % The white line.
    y2(1:15,1:30,:) = 0;           % Background for displaying count
    
    y2(1:15,32:62,:) = 0;           % Background for displaying count
    
    y2(1:15,64:94,:) = 0;           % Background for displaying count
    
    
    
    
  y2 = step(textInserter, image,velocitiesppfkmperhor,uint32(centroidmat));
    
    
    image_out_real = step(htextins,y2,count); 
    
   % image_out = step(htextinscurrentframe, y3, count);
    
    
    
 Generate the coordinate points for plotting motion vectors:
    if firstTime
      [R C] = size(of);            % Height and width in pixels
      RV = borderOffset:decimFactorRow:(R-borderOffset);
      CV = borderOffset:decimFactorCol:(C-borderOffset);
      [Y X] = meshgrid(CV,RV);
      firstTime = false;
    end
 
    % Calculate and draw the motion vectors.
    tmp = of(RV,CV) .* MotionVecGain;
    lines = [X(:)';Y(:)';X(:)' + imag(tmp(:))';Y(:)' + real(tmp(:))'];
 

%show lines:
    mv_video = step(hshapeins2, image, lines);
 
 

%Show Results :
    
    step(hVideo1, image);          % Display Original Video
    step(hVideo2, mv_video);       % Display video with motion vectors
    step(hVideo3, th_image);       % Display Thresholded Video
    step(hVideo4, image_out);      % Display video with bounding boxes
    step(hVideo5, image_out_real);      % Display the real velocity
end


