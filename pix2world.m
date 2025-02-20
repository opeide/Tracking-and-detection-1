
function [world_coords, correct] = pix2world(pix_coords, IntrinsicMatrix, worldOrientation, worldLocation, face, vertex)
    %Convert point in pixels to point on model in world coords
    correct = 0;
    world_coords = nan;
    
    %pixel coords, make homogenous
    point_homo = cat(2, pix_coords, 1);
    
     %ray to point in camera coords
    ray_origin_cam = 0;
    ray_direction_cam = inv(IntrinsicMatrix')*point_homo';
    
    %convert to world coord
    ray_origin_world = ray_origin_cam + worldLocation;
    ray_direction_world = inv(worldOrientation)*ray_direction_cam;
    
    %intersections with model faces. points 123 make up a triangle face
    points1 = vertex(face(:, 1), :);
    points2 = vertex(face(:, 2), :);
    points3 = vertex(face(:, 3), :);
     
    [intersect, t, u, v, xcoor] = TriangleRayIntersection(ray_origin_world, ray_direction_world, points1, points2, points3);
    
    for i = 1:length(intersect)
        if intersect(i) == 1
            vect1 = points3(i, :) - points2(i, :);
            vect2 = points1(i, :) - points2(i, :);
            face_dir = cross(vect1, vect2);
            if face_dir*ray_direction_world < 0
                world_coords = xcoor(i, :);
                correct = 1;
            end
        end
    end
    %intersect_point = ray_origin_world + ray_direction_world'*t