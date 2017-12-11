data = load('worldToImageCorrespondences.mat');

[worldOrientation,worldLocation] = estimateWorldCameraPose(...
     data.imagePoints,data.worldPoints,data.cameraParams);
 
 
  pcshow(data.worldPoints,'VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
 hold on
 plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
     worldLocation);
 hold off