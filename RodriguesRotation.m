function rotatedVector = RodriguesRotation(vector,theta,axis)
    % Compute the cross product between the axis and the vector
    crossProduct = cross(axis, vector);
    
    % Compute the dot product between the axis and the vector
    dotProduct = dot(axis, vector);
    
    % Compute the rotated vector using the Rodrigues rotation formula
    rotatedVector = vector * cos(theta) + crossProduct * sin(theta) + axis * dotProduct * (1 - cos(theta));
end