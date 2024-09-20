import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 0)  # 只启用深度流

pipeline_profile = pipeline.start(config)
device = pipeline_profile.get_device()
depth_sensor = device.first_depth_sensor()
intrinsics = depth_sensor.get_stream_profiles()[0].as_video_stream_profile().get_intrinsics()

intrinsic_matrix = [
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1]
]

print("Intrinsic Matrix:")
for row in intrinsic_matrix:
    print(row)
