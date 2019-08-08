# Camera Adjustment Processor

This node provides many filtering operations for a camera topic. Filters can be described using either rosparams local to the node or by using dynamic reconfigure while the node is running. Generally, rosparams are used to set default values which can then be tuned with dynamic reconfigure on the fly.


## Required ROS Params

`~cam_topic`: name of camera topic this node subscribes to

## Other ROS Params

`~resize`: relative resize. Input should be between 0.0 and 1.0

`~enable_less_color`: in HSV color space, subtract `saturation * less_color_mux` from value. This provides a simple method for flattening brightly colored areas of an image.

`~less_color_mux`: The amount to apply the less_color filter above. 

`~enable_clahe`: enables and disables clahe on the image

`~clahe_clip`: Sets the clip limit for the clahe operator using OpenCV's `setCliplimit`

`~enable_color_correct` enables and disables color correction

`~cc_alpha`: Alpha parameter for adjusting brightness and contrast (see https://docs.opencv.org/3.4/d3/dc1/tutorial_basic_linear_transform.html)

`~cc_beta`: Beta parameter for adjusting brightness and contrast (see https://docs.opencv.org/3.4/d3/dc1/tutorial_basic_linear_transform.html)

`~enable_sharpen`: enables and disables image sharpening

`~sharp_weight`: Parameter for the sharpen parameter

`~sharp_kernel`: Parameter for the kernel parameter

## Example Launch File

```
<launch>
    <node pkg="ltu_actor_inputprocess_camadjust" name="cam_repub_sunny" type="cam_repub">
	<param name="cam_topic" type="string" value="/camera/image_raw" />
	<param name="resize" type="double" value="1" />
	<param name="enable_less_color" type="bool" value="False" />
	<param name="less_color_mux" type="double" value="1" />
	<param name="enable_clahe" type="bool" value="False" />
	<param name="clahe_clip" type="int" value="2" />
	<param name="enable_color_correct" type="bool" value="False" />
	<param name="cc_alpha" type="double" value="1" />
	<param name="cc_beta" type="double" value="0" />
	<param name="enable_sharpen" type="bool" value="False" />
	<param name="sharp_weight" type="double" value="0.5" />
	<param name="sharp_kernel" type="int" value="1" />
    </node> 
</launch>
```
