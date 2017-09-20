# Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./examples/undistort_output.png "Undistorted"
[image2]: ./examples/distortion_correction.png "Road Transformed"
[image3]: ./examples/binary_combo_example.png "Binary Threshold Combo"
[image4]: ./examples/warped_birds_eye.png "Warp Example"
[image5]: ./examples/straight_lane_birdseye.png "Straight Warp Example"
[image6]: ./examples/findInitialLaneInImage_example.png "Window Lane Finding"
[image7]: ./examples/results_example.png "Results"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Writeup, Notebooks, and Files

This Writeup serves as a guide to how my projects addresses the items called out in the Rubric.  The [AdvancedLaneFinding.ipynb](https://github.com/Merberg/CarND-Advanced-Lane-Lines/blob/master/AdvancedLaneFinding.ipynb) notebook houses the source to calibrate the camera and the pipeline used to detect the lane lines within images.  Peppered between functions are cells (noted as `Testing/Plotting Cell(s):`) that are inserted to test functionality.  Test images and found line result videos can be found in test_images and output_images respectively.

### Camera Calibration
The _Camera Calibration_ notebook cells perform the following functions:
1. Read in the images (RGB)
2. Convert to grayscale
3. Use the OpenCV `findChessboardCorners` to locate the 9x6 corners on the image
4. Store valid corners (imagePoints) and their corresponding world points (worldPoints)
5. Use the image and world points as parameters for the OpenCV `calibrateCamera`
6. Save the resulting camera matrix and distortion coefficients for use by the pipeline with the OpenCV `undistort` method.

This is an example of a calibration test image that has had its distortion corrected:

![Undistorted][image1]

### Pipeline (single images)


#### 1. Distortion-corrected

The first step in the pipeline, the `Functions for undistortion` section, uses the camera calibration matrix and the cv2.undistort() function to correct an image like so:

![Step 1 Distortion-Correction][image2]

#### 2. Binary Image Threshold Correction

The `Functions to create filtered binary images` section contains the functions utilized for performing binary image corrections.  Initially, [ThresholdTesting.ipynb](https://github.com/Merberg/CarND-Advanced-Lane-Lines/blob/master/ThresholdTesting.ipynb) was used to experiment with various Sobel threshold and colorspace techniques.  The final threshold combination used to produce the following filtered images is:

```
(Sobel Magnitude & Sobel Direction) | Saturation
```

![Step 2 Binary Threshold Combo][image3]

#### 3. Perspective Transformation

![Step 3 Birds Eye Transformation][image4]

The triptych above shows the undistorted image and its binary threshold and colored birds eye transformations.  To perform this transformation, source and destination coordinates are required.  These imperical values were derived by first measuring points on a undistorted image of a straight lane, then adjusting the source points to have the lines as straight as possible in the resulting transform.

![Step 3 Birds Eye Straight][image5]

```python
birdsEyeOffset = np.absolute(LANE_WIDTH_pixel - 1280)/2
birdsEyeSrc = np.float32([(605, 442),
                          (674, 442),
                          (1068, 685),
                          (240, 685)])
birdsEyeDst = np.float32([[birdsEyeOffset, 0],
                          [1280-birdsEyeOffset, 0],
                          [1280-birdsEyeOffset, 720],
                          [birdsEyeOffset, 720]])
```

#### 4. Lane Line Identification

The `Functions to find the lane lines within the images` section has the following 4 definitions:

| Function | Purpose |
|----------|---------|
| findLaneStarts | Calculates the histogram of a binary image to identify two maximum indicies within the peak regions; one for the left land starting point and another for the right lane.  |
| findLaneInWindow | Frames a portion of an image to locate the nonzero pixels contained within.  This function also determines the center x index of the next window with a simple mean trend. |
| findInitialLaneInImage | Uses the two functions above to loop through the left or right half of an image and capture valid indices.  A second degree polynomial is then fit to values of these indices.  The coefficients of this polynomial create a line to approximate the lane line.  Examples of this windowed approach is shown below the table. |
| findLaneInImage | Using the parameterized coefficients, creates a polynomial window within half of an image to search out included values.  A second degree polynomial is then refit to this new dataset. |

![Step 4 Window Lane Finding][image6]

#### 5. Curvature and Location Estimation

The second degree polynomial coefficients are again used to find information about the image.  In this case, they are used to generate points along the line that are then converted to real world values with the following constants:

```python
LANE_LENGTH_m = 30
LANE_WIDTH_m = 3.7
LANE_WIDTH_pixel = 700
M_PER_PIXEL_y = LANE_LENGTH_m/720
M_PER_PIXEL_x = LANE_WIDTH_m/LANE_WIDTH_pixel
```
The formula for the radius of curvature at any point _x_ for the curve _y = f(x) = the lane line's second degree polynomial_ can be used to determine the radius in meters.
To track the car's placement within the lane, offsets from image center (assuming center camera mounting) are tracked.  These approximate values help orient lane calculations in the real world.

#### 6. Marking Lanes on Images

##### LaneLine Class

To mark up the lanes on an image information must be tracked.  The `LaneLine` class acts as a container for storing data applied to multiple images.  Some `LaneLine` members, like the polynomial coefficients, are updated using moving averages to assist in error correction.  It also has methods that utilize the lane indentification and real world estimate functions noted in sections above.  It is in the `adjust` method of this class that the two lanes work together to suppliment information.

Once two `LaneLine` classes are populated, a polygon can be created that marks the lane boundaries.  This polygon is subjected to a reverse birds eye transform and then overlaid on a undistorted, color image.  Text noting the radius and center offset is also added.

![Step 6 Image Markup][image7]

---

### Lane Finding Pipeline (video)

The steps noted above a sequenced together to form the final pipeline:
1. Distortion correction
2. Binary image threshold creation
3. Perspective transformation
4. Lane line identification
5. Curvature and location estimation
6. Image Markup

```
leftLane = LaneLine(True)
rightLane = LaneLine(False)

def findLanes(img):
    
    undistorted = undistortImage(img)
    binary = combineThresholds(undistorted)
    warped = warpToBirdsEye(binary)
    
    leftLane.findInImage(warped)
    rightLane.findInImage(warped)
    leftLane.adjust(rightLane)
    rightLane.adjust(leftLane)
    
    resultImg = createLaneDrawing(warped, undistorted, leftLane, rightLane)
    return resultImg
```

The pipeline in action: [Lane Finding Results Video](./project_video_results.mp4)

---

### Discussion

I feel my approach is fragile as evident by its performance with the challenge videos.  First, the imperical coordinates used for the birds eye transformation fail with modifications to images such as camera mounting, camera movement/bouncing, and resolution.  This could be addressed using geometry and known properties of the camera.  Additional cropping could also be applied to remove noise from adjacent lanes and shoulders.  Secondly, there could be more interaction/coordination between the `LaneLine` instances using real world attributes for error correction.  I belive this could prevent the lane from jumping onto the center barrier in the challenge video.  And last, accounting for a know trajectory (ie lanes have predictive behavior) could correct harsh light conditions like what is evident in the harder challenge video.
