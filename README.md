# points_testing2
<p>A program building off of the concept from my other repo points_testing. Implements the Ramer Douglas Peucker algorithm to downsample a set of Poses (containing Position and Orientation) in order to simplify a path bound by two different constraints. These constriants are the maximum perpendicular distance that the original points can vary from the corrected points, and the maximum angle about any axis that the original orientations can vary from the corrected orientations.</p>
<p>This code is set up as a ROS 2 Service.<br>This service has 3 requests:
<ul>
  <li><code>input_file (string)</code> - The PoseArray to be corrected</li>
  <li><code>epsilon (float64)</code> - The maximum amount that the perpendicular distance between the original points and the corrected points can vary</li>
  <li><code>angle_threshold (float64)</code> - The maximum amount about any axis that the original orientations can vary from the corrected orientations</li>
</ul>

The service has 1 response:
<ul>
  <li><code>corrected_poses (geometry_msgs/PoseArray)</code> - The downsample PoseArray</li></ul>
  
When the service is called, the code follows this logic to downsample the poses:

1. The code first finds the maximum perpendicular distance that the curve is from a line drawn between the start and end points, and then the maximum rotation about an axis for that range
2. The code then checks if the maximum distance is greater than the epsilon value, if it is, the list of poses is split at that index, and steps 1-2 will be repeated until a pose is found with a maximum distance of less than epsilon
3. Once a segment has been determined as valid for distance from the original points, the segment is then evaluated for how much the orientation varies from the original set of poses. If a point is found where the rotation relative to each axis is greater than the threshold, the pose list will be split at that point's index, and steps 1-3 will be repeated until an orientation is found that is less than the threshold
4. Once all positions are checked and valid, the code will move on to the next segment until the end of the list is reached and the result array is outputted

<br>
Here is an example of a raster dataset that is downsampled. The orientations of the original raster were set to vary up to 40 degrees
<img width="100%" alt="Example of code that downsamples a large set of poses" src="https://github.com/SamanthaSmith04/points_testing2/assets/82625799/177f55f9-3371-415f-9e64-0274fa743438">
Original PoseArray: 2995 poses, Corrected PoseArray: 8 poses<br>
Time to complete: ~1 second<br>
Epsilon: 0.05m<br>
Angle Threshold: 35 degrees<be>
<img width="695" alt="Console info for delta values" src="https://github.com/SamanthaSmith04/points_testing2/assets/82625799/793ff18f-74d5-439c-b3bc-5a37df88a68f">
Note: The first section of orientations shows the difference between the correction poses, the second set with the Delta values shows the maximum orientation from the original dataset between the two poses

</p>
