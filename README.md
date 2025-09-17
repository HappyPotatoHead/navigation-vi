# Navigation System for The Visually Impaired Using QR Code 

A cross-platform C++ application using OpenCV and ZBar to detect QR codes, map coordinates, and guide users to destinations in real time. 

# Features

- Real-time QR Code detection using ZBar
- Colour filtering - Red, Green, Blue
- Map-based navigation with room connections
- Cross-platform build with CMake (Linux, Windows, macOS)

# Requirements

- C++17 compiler
- CMake 3.10+
- OpenCV 4.11
- ZBar library

# Installing Dependencies

## Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install libopencv-dev libzbar-dev cmake g++
```

## Windows

Proceed to OpenCV's and ZBar's official website to download their library. 

- Add `include/` to your compiler include path.
- Add `.lib` to your linker input
- Place `.dll` next to your executbale or in your PATH.

> Visual Studio has a GUI dedicated to configuring this. 

Install CMake for Windows

# Project Structure

```bash
navigation-vi/
│
├── core/                # AppController, UIManager
├── modules/             # QRDetector, QRReader, CoordinateMapSystem, RouteGuidance
├── utils/               # rooms.txt, connections.txt
├── CMakeLists.txt       # Cross-platform build config
├── main.cpp
├── README.md
└── .gitignore
```

# Building the Project

## Linux/macOS

```bash
git clone https://github.com/HappyPotatoHead/navigation-vi.git
cd navigation-vi
mkdir build && cd build
cmake ..
make ./navigation
```

## Windows (Visual Studio with CMake)

1. Clone the repo
2. Open the folder in Visual Studio
3. Configure CMake:
    - if `pkg-config` is unavailable, set
```bash
ZBAR_INCLUDE_DIRS = C:/path/to/zbar/include
ZBAR_LIBRARIES    = C:/path/to/zbar/lib/zbar.lib
```
4. Build and run

# Configuration

- Place `rooms.txt` and `connections.txt` inP the `utils/` folder.
- Run the program from the **project root** so relative paths work
- On startup, enter:
    - Target QR Colour (`red`, `green`, `blue`, or `none`)
    - Destination room ID

# Example Run

```bash
./navigation
Target QR colour: red
Destination room ID: 102
```

# Troubleshooting
- **ZBar** not found: 
    - Install `libzbar-dev` (Linux)
    - Set `ZBAR_INCLUDE_DIRS` and `ZBAR_LIBRARIES` manually (Windows)
        - Visual Studio has a GUI dedicated to configure this (under project setting)
- **OpenCV** not found
    - Ensure `pkg-config` can find it (Linux)
    - Set `OpenCV_DIR` in CMake.
        - Visual Studio has a GUI dedicated to configure this (under project setting)
- **Relative path errors**
    - Make sure you run from the project root.

> If all else fails, extract the code and rebuild the environment