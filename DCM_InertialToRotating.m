function DCM = DCM_InertialToRotating(frameRotationRate,t)

DCM = [cos(frameRotationRate*t) sin(frameRotationRate*t) 0;
          -sin(frameRotationRate*t) cos(frameRotationRate*t) 0;
          0 0 1];
end