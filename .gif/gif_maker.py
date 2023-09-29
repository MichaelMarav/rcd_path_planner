from moviepy.editor import VideoFileClip


videoClip = VideoFileClip("/home/michael/Videos/RCD.mp4")
videoClip.write_gif("RCD.gif",fps = 10)