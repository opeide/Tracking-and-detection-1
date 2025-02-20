run('~/matlab2017b/toolbox/vlfeat-0.9.20/toolbox/vl_setup');

%This function has to be run first. Commented out, snice it only needs to
%be run once
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

for i=1:nImgs
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
   
    plot(pix_match_loc(1,inliersID), pix_match_loc(2,inliersID), 'b+');
   
   pos = worldToImage(cameraParams , inv(worldOrientation), -worldOrientation*worldLocation', vertex);
   plot(pos(:,1), pos(:,2), 'g*');

    
   
   
   %non linear minimizer of errorFunc
   M = model_match_loc;
   m = pix_match_loc;
   nPoints = size(M);
   nPoints = nPoints(2);
    f = @(x)errorFunc(x, IntrinsicMatrix', M, m, nPoints); 
    R0 = rotationMatrixToVector(inv(worldOrientation));
    T0 = -worldOrientation*worldLocation';
    x0 = [R0' T0];
    
    x = fminsearch(f, x0)
    
    R = x(:,1)
    T = x(:,2)
    R = rotationVectorToMatrix(R)
    pos = worldToImage(cameraParams , inv(worldOrientation), -worldOrientation*worldLocation', vertex);
    plot(pos(:,1), pos(:,2), 'Om');
    waitfor(h)
end