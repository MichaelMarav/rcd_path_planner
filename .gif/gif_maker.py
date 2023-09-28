from moviepy.editor import VideoFileClip


videoClip = VideoFileClip("RCD.mp4")
videoClip.write_gif("RCD.gif")