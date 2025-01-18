# OceanSensorv0.1

## OceanSensorNES Board Integration Guide

This repository’s `main` branch contains the hardware schematic and device tree definitions for the OceanSensorNES board using Zephyr. To use these board definitions in a new Zephyr application, follow the steps below.

---

## Use the Board Design in Your Application

### Clone the Board Design Repository

First, clone the repository that contains the board design. Open your terminal and run:

```bash
git clone https://github.com/LMY-Mengyao/OceanSensorNES.git
```

### Notes
- Update your application’s configuration to reference the new board files if necessary.
- Ensure your Zephyr project’s `CMakeLists.txt` or build configuration is aware of the additional board directory.
- For any changes to the board schematic or device tree, update the files in the `boards/OceanSensorNES` directory of your application accordingly.
- If you encounter issues, refer to the [Zephyr documentation](https://docs.zephyrproject.org/latest/) or the repository’s issues section for support.


