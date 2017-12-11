[vertex, face] = read_ply('data/model/teabox.ply');

FX = 2960.37845;
FY = FX;
CX = 1841.68855;
CY = 1235.23369;

IntrinsicMatrix = [FX 0 0; 0 FY 0; CX CY 1];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);

trainImgs = dir('data/images/init_texture/*.JPG');
nImgs = length(trainImgs);

for i=2:2
   currentFilename = trainImgs(i).name;
   currentImg = imread(strcat('data/images/init_texture/', currentFilename));
   currentImg = rgb2gray(currentImg);
   imshow(currentImg);
   [xf, yf] = getpts;
   close all
   
   nImgPoints = length(xf);
   
   cropSize = 15;
   
   cornerPoint = struct('img', cell(1, nImgPoints));
                    
   cornerPos = zeros(nImgPoints, 2);
   cornerIndex = zeros(nImgPoints, 1);
                    
   h = figure;
   for j=1:nImgPoints
       xi = int32(xf(j) + 0.5);
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
   waitfor(h);
   
   h = figure;
   imshow(currentImg);
   hold on;
   
   for j=1:nImgPoints
       x = cornerPos(j, 1);
       y = cornerPos(j, 2);
       plot(x, y, 'r*');
       text(double(x)+10, double(y)+10, cellstr(num2str(j)), 'FontSize',32);
   end
   
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
   
   cornerWorldPos = vertex(indecies,:);
   
   [worldOrientation,worldLocation] = estimateWorldCameraPose(cornerPos,cornerWorldPos,cameraParams,...
       'MaxNumTrials', 10000, 'Confidence', 95, 'MaxReprojectionError', 50);
   %error when max reprojection under 1784
end