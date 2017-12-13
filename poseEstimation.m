

%generate model texture
%[model_desc, model_desc_loc] = generate3Dmodel();

[vertex, face] = read_ply('data/model/teabox.ply');

%init camera
FX = 2960.37845;
FY = FX;
CX = 1841.68855;
CY = 1235.23369;
IntrinsicMatrix = [FX 0 0; 0 FY 0; CX CY 1];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);

%generating camera poses for all images in images/detection
detectImgs = dir('data/images/detection/*.JPG');
nImgs = length(detectImgs);

highestNrInliers = 0;
highestNrInliersImgID = 0;

%filtering out the best hypothesis for each pose
%where to ransac?

for i=16:nImgs
   currentFilename = detectImgs(i).name;
   currentImg = imread(strcat('data/images/detection/', currentFilename));
   currentImg = rgb2gray(currentImg);
   h = figure(1);
   imshow(currentImg);
   hold on
   
   %find features in image
    peak_thresh = 5;
    currentImg = single(currentImg);
    [frame, desc] = vl_sift(currentImg, 'PeakThresh', peak_thresh);
    num_features = size(frame);
    num_features = num_features(2);
    
    %match found features with known model meatures
    [matches, scores] = vl_ubcmatch(desc, model_desc);
%     indexes = matches(1, :);
%     model_indexes = matches(2, :);
    
    %get best topPart (fraction of all) matches
    topPart = 0.9;
    [sortedValues,sortIndex] = sort(scores(:),'ascend');  %# Sort the values in
    bestMatches = sortIndex(1:topPart*length(sortIndex));
    indexes = matches(1, bestMatches);
    model_indexes = matches(2, bestMatches);
    plot(frame(1, indexes), frame(2, indexes), 'r+')
    

   %2d and 3d position of found features
   pix_match_loc = frame(1:2, indexes); 
   model_match_loc = model_desc_loc(:, model_indexes);

   %compute camera pose from 3d-2d correspondences
   [worldOrientation, worldLocation, inliersID] = estimateWorldCameraPose(pix_match_loc', model_match_loc', cameraParams, ...
       'MaxNumTrials', 100000, 'Confidence', 99, 'MaxReprojectionError', 2);
   
   pos = worldToImage(cameraParams , inv(worldOrientation), -worldOrientation*worldLocation', vertex);
   plot(pos(:,1), pos(:,2), 'g*');
   waitfor(h)
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