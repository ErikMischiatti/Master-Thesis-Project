import pyrealsense2 as rs

class RealSenseCamera:
    def __init__(self):
        """Initialize and configure the RealSense camera."""
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline.start(self.config)

    def get_frames(self):
        """Get synchronized RGB and depth frames."""
        align_to = rs.stream.color
        align = rs.align(align_to)

        frames = self.pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        return color_frame, depth_frame

    def stop(self):
        """Stop the RealSense pipeline."""
        self.pipeline.stop()