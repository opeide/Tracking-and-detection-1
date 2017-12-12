%init camera
FX = 2960.37845;
FY = FX;
CX = 1841.68855;
CY = 1235.23369;
IntrinsicMatrix = [FX 0 0; 0 FY 0; CX CY 1];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);

%load 3d model
[vertex, face] = read_ply('data/model/teabox.ply');

%load training images
trainImgs = dir('data/images/init_texture/*.JPG');
nImgs = length(trainImgs);

%Map training images to 3d model
for i=2:2
   currentFilename = trainImgs(i).name;
   currentImg = imread(strcat('data/images/init_texture/', currentFilename));
   currentImg = rgb2gray(currentImg);
   imshow(currentImg);

   
   
   %let user select corners
   [xf, yf] = getpts;
   close all
   

   
   
   nImgPoints = length(xf);
   %init empty
   cornerPoint = struct('img', cell(1, nImgPoints));                  
   cornerPos = zeros(nImgPoints, 2);
   cornerIndex = zeros(nImgPoints, 1);
   
   %pixels radius (square) to search for corner
   cropSize = 15;
   %increase corner accuracy with harris detector
   h = figure;
   for j=1:nImgPoints
       xi = int32(xf(j) + 0.5);     %rund opp
       yi = int32(yf(j) + 0.5);
       
       cropRect = [xi - cropSize, yi - cropSize, 2*cropSize, 2*cropSize];
       cornerPoint(j).img = imcrop(currentImg, cropRect);
       corners = detectHarrisFeatures(cornerPoint(j).img);
       corner = corners.selectStrongest(1);
       localPos = corner.Location;
       cornerPos(j, :) = [localPos(1) + single(cropRect(1))  localPos(2) + single(cropRect(2))];
       
       
       subplot(3, 3, j);
       imshow(cornerPoint(j).img);
       hold on;
       plot(corner);
       hold off;
   end
   waitfor(h);  %until figure closed
   
   
   
   h = figure;
   imshow(currentImg);
   hold on;
   
    %plot initial corner placement
   for j=1:nImgPoints
       x = cornerPos(j, 1);
       y = cornerPos(j, 2);
       plot(x, y, 'r*');
       text(double(x)+10, double(y)+10, cellstr(num2str(j)), 'FontSize',32);
   end
   %correct corner placement
   indecies_input = inputdlg('Enter vertex indecies:', 'Vertecies', [1 50]);
   indecies = str2num(indecies_input{:});
   
   assert(length(indecies) == nImgPoints, 'Wrong number of indecies');
   
   for j=1:length(indecies)
       cornerIndex(j) = indecies(j);
   end
   close all
   
   h = figure;
   imshow(currentImg);
   hold on;
   
   
   % Show image with mesh indecies
   for j=1:nImgPoints
       x = cornerPos(j, 1);
       y = cornerPos(j, 2);
       plot(x, y, 'r*');
       text(double(x)+10, double(y)+10, cellstr(num2str(cornerIndex(j))), 'FontSize',32);
   end
   waitfor(h);
   
   %PnP to decide camera-world mapping
   cornerWorldPos = vertex(indecies,:);
  [worldOrientation,worldLocation] = estimateWorldCameraPose(cornerPos,cornerWorldPos,cameraParams,...
   'MaxNumTrials', 10000, 'Confidence', 95, 'MaxReprojectionError', 50);
   
    %extract ROI
    figure(3);
    corners_x = cornerPos(:,1)
    corners_y = cornerPos(:,2)
    poly_indexes = convhull(corners_x, corners_y);
    minx = min(corners_x(poly_indexes));
    miny = min(corners_y(poly_indexes));
    maxx = max(corners_x(poly_indexes));
    maxy = max(corners_y(poly_indexes));
    rect = [minx, miny, maxx-minx, maxy-miny];
    img_roi = imcrop(currentImg, rect);
    imshow(img_roi);
    hold on;
    
    %Detect features in ROI
    num_features = 100;
    %TODO: use the library listed in exercise
    corners = detectHarrisFeatures(img_roi);
    corners = corners.selectStrongest(num_features)
    for k = 1:num_features        
        pos = corners.Location(k, :)
        pos(1)
        pos(2)
        plot(pos(1), pos(2), 'b+');
    end
    
    %Map features to 3d model
    
   
end








