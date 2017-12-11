[vertex, face] = read_ply('data/model/teabox.ply');
IntrinsicMatrix = [2960.37845 0 0; 0 2960.37845 0; 1841.68855 1235.23369 1];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);

img = imread('data/images/init_texture/DSC_9743.JPG');

R = rotz(5)*roty(5)*rotx(5);
t = [0.0 0 -0.7];

close all

pos = worldToImage(cameraParams, R, t, vertex);
imshow(img)
hold on
plot(pos(:,1), pos(:,2), 'r*');

figure

[worldOrientation,worldLocation] = estimateWorldCameraPose(pos,vertex,cameraParams,...
       'MaxNumTrials', 10000, 'Confidence', 99, 'MaxReprojectionError', 0.1);

pcshow(vertex,'VerticalAxis','Y','VerticalAxisDir','down', ...
'MarkerSize',300);
hold on
plotCamera('Size',0.03,'Orientation',worldOrientation,'Location',...
worldLocation);
hold off


pos = worldToImage(cameraParams, inv(worldOrientation), worldLocation, vertex);
figure
imshow(img)
hold on
plot(pos(:,1), pos(:,2), 'r*');
 