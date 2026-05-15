# BlueOS Stereo Processor Extension

A ROS2-based stereo vision processing extension for BlueOS vehicles. This extension enables real-time 3D reconstruction and SLAM capabilities for autonomous underwater vehicles.

## Features

- Real-time stereo vision processing
- ROS2 Jazzy based
- Stereo matching and depth estimation
- 3D mapping and SLAM capabilities
- Persistent data storage on vehicle

## Hardware Requirements

- Stereo camera pair (or compatible camera system)
- BlueOS-compatible onboard computer (Raspberry Pi 4+ or equivalent)
- Sufficient storage for 3D map data

## Installation

1. Install from BlueOS Extensions Manager under the Bazaar
2. Select your preferred hardware architecture (arm64 or armv7)
3. Configure permissions if prompted
4. Enable the extension

## Configuration

Edit the extension configuration in the Extensions Manager to customize:
- ROS node launch parameters
- Camera calibration files
- Mapping output directory
- DDS communication settings

## ROS2 Communication

The extension runs ROS2 nodes that can communicate with other services:
- **DDS Network**: Uses standard ROS2 DDS discovery (ensure network allows UDP 7400+)
- **Topics**: Published depth maps, odometry, and 3D point clouds
- **Data Storage**: Maps and calibration data stored in `/usr/blueos/extensions/stereo-processor/data/`

## Troubleshooting

### Extension won't start
Check logs in BlueOS Extensions Manager:
1. Open Extensions Manager
2. Select this extension
3. View logs for error messages

### ROS2 nodes not discovering other services
Verify:
- Network connectivity between vehicle and topside
- DDS middleware configuration (check environment variables)
- No firewall blocking UDP ports 7400-7410

### Camera not detected
- Check `/dev/video*` devices are accessible
- Verify camera permissions in extension configuration
- Update camera calibration files if needed

## Development

For development or custom modifications:

```bash
# Clone the repository
git clone https://github.com/yourusername/mapping4

# Build locally
cd mapping4/.devcontainer/pi
docker build -t blueos-stereo-processor:dev .

# Test with Docker
docker run --rm blueos-stereo-processor:dev
```

## Support

For issues or feature requests, please:
- Check [GitHub Issues](https://github.com/yourusername/mapping4/issues)
- Email: support@example.com

## License

[Your License Here]
