# InputProcess-CamAdjust

## Example Launch

```
<launch>

    <node pkg="ltu_actor_inputprocess_camadjust" name="cam_repub_sunny" type="cam_repub">
	<param name="cam_topic" type="string" value="/camera/image_raw" />
	<param name="resize" type="double" value="1" />
	<param name="enable_clahe" type="bool" value="False" />
	<param name="clahe_clip" type="int" value="2" />
	<param name="enable_color_correct" type="bool" value="False" />
	<param name="cc_alpha" type="double" value="1" />
	<param name="cc_beta" type="double" value="0" />
	<param name="enable_sharpen" type="bool" value="False" />
	<param name="sharp_weight" type="double" value="0.5" />
	<param name="sharp_kernel" type="int" value="1" />
    </node> 

    <node pkg="ltu_actor_inputprocess_camadjust" name="cam_repub_cloudy" type="cam_repub">
	<param name="cam_topic" type="string" value="/camera/image_raw" />
	<param name="resize" type="double" value="1" />
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
