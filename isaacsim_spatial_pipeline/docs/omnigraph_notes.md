# OmniGraph Notes

The ROS2 graph scripts are placeholders until Milestone 1 validation is done.

Authoring order:

1. Open the warehouse stage.
2. Verify Spot, LiDAR, camera, and IMU prim paths.
3. Enable the Isaac Sim ROS2 bridge extension.
4. Create `/clock` publishing.
5. Create TF publishing from verified prims.
6. Create sensor publishing graphs.
7. Validate ROS2 topics, message types, frame IDs, and timestamps.

Do not save a ROS2-ready USD until the topic and TF contract is documented.
