# Demo media

Once the sim + nav + vision pieces are working end to end, record a short
(15-30s) screen capture of:

1. Robot driving a waypoint route through the greenhouse aisles in Gazebo
2. Camera view with ArUco detection overlay as it approaches a table
3. The lateral alignment happening (robot nudging into offset)
4. RViz side-by-side showing the costmap/path

Convert to a GIF (e.g. `peek`, or `ffmpeg` + `gifski`) under ~8MB and save
here as `demo.gif`, then update the README embed.

```bash
ffmpeg -i demo_raw.mp4 -vf "fps=12,scale=800:-1:flags=lanczos" -c:v gif demo.gif
```
