# Camera Adjustment Processor

Takes camera input and processes for route nodes

## Required ROS Params

`~cam_topic` name of camera topic this node subscribes to

## Other ROS Params

`~resize`

`~enable_less_color`

`~less_color_mux`

`~enable_clahe` enables and disables clahe on the image

`~clahe_clip`

`~enable_color_correct` enables and disables color correction

`~cc_alpha`

`~cc_beta`

`~enable_sharpen` enables and disables image sharpening

`~sharp_weight`

`~sharp_kernel`

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
