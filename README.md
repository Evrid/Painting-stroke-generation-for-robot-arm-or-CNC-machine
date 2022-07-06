# Painting-stroke-generation-for-robot-arm-or-CNC-machine

So the code take in a picture, make strokes, sort the strokes by color (because we don't want wash the pen too often), display number of strokes for each color, add the add water and color procedure of a few strokes interval. 

After I load all strokes for color01, Roboguide became not responding (guess too much strokes), so I just used first 300 lines and the result can be seen
https://www.youtube.com/watch?v=4dgLnZiYgOo

Output in format of: x,y,z,yaw,pitch,roll

One output file I got has 200k lines.

To import the output to Roboguide, see
https://www.youtube.com/watch?v=k4gOBWY9oB4

% parts code from Painterly Rendering with Curved Brush Strokes of Multiple Sizes
%https://github.com/fionazeng3/Painterly-Rendering-with-Curved-Brush-Strokes-of-Multiple-Sizes

