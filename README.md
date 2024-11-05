# ur10e_rg2

## Prerequisites

### Unity Installation
1. Download and install [Unity Hub](https://unity.com/download)
2. Install Unity Editor version 2021.3.45f1 through Unity Hub
   - Open Unity Hub
   - Go to "Installs" -> "Install Editor"
   - Select version 2021.3.45f1
   - Make sure to include "Windows Build Support" in the installation modules

## Project Setup

### Clone Repository
```bash
git clone https://github.com/PhuNguyenPT/ur10e_rg2.git
```

## Open Project in Unity

1. Open **Unity Hub**.
2. Click **Open** -> **Add project from disk**.
3. Navigate to the cloned repository folder and select it.
4. Open the project with **Unity 2021.3.45f1**.

## Required Packages

Install the following packages through the **Package Manager**:

1. Open **Window -> Package Manager** in Unity.
2. Click the **+** button in the top-left corner.
3. Select **Add package from git URL**.
4. Add each of the following packages one by one:

   - **ROS TCP Connector**  
     URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.7.0`
     
   - **ROS TCP Visualizations**  
     URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations#v0.7.0`

   - **URDF Importer**  
     URL: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2`

## Important Notes
<!--
- For **glTFast**, ensure shader variants are properly included in your builds by following their documentation.
- The **URDF Importer** requires the proper setup of mesh file locations for your project.
-->
## Getting Started

[Add your project-specific instructions here]
