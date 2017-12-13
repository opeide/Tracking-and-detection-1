





%generating camera poses for all images in images/detection
detectImgs = dir('data/images/detection/*.JPG');
nImgs = length(detectImgs);

highestNrInliers = 0;
highestNrInliersImgID = 0;

%filtering out the best hypothesis for each pose
%where to ransac?

for i=1:1
   currentFilename = detectImgs(i).name;
   currentImg = imread(strcat('data/images/detection/', currentFilename));
   currentImg = rgb2gray(currentImg);
   figure(1);
   imshow(currentImg);

%    %compute camera orientation and location
%    [worldOrientation, worldLocation, inliersID] = estimateWorldCameraPose(data.imagePoints, data.worldPoints, data.cameraParams);
%    
%    %plot
%    pcshow(data.worldPoints,'VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',30);
%    hold on
%    plotCamera('Size',10,'Orientation',worldOrientation,'Location', worldLocation);
%    hold off
%    
%    if(highestNrInliers < length(inliersID))
%        highestNrInliers = length(inliersID);
%        higestNrInliersImgID = i;
%    end
end