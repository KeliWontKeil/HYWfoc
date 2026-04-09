# Third-Party Notices

This project is released under the MIT License for original HYWfoc code.

Some files in the repository are third-party vendor components under their own licenses. Those component licenses apply to the corresponding files and take precedence over the repository MIT license for those files.

## Component Inventory

1. Arm CMSIS (legacy CMSIS core headers)
- Path scope: `examples/GD32F303_FOCExplore/software/Firmware/CMSIS/`
- Typical license style in headers: 3-clause BSD-like text
- Example file: `examples/GD32F303_FOCExplore/software/Firmware/CMSIS/core_cm4.h`

2. Arm CMSIS (modern helper headers)
- Path scope: `examples/GD32F303_FOCExplore/software/.cmsis/include/`
- Typical license style in headers: Apache-2.0 (`SPDX-License-Identifier: Apache-2.0`)
- Example file: `examples/GD32F303_FOCExplore/software/.cmsis/include/cmsis_gcc.h`

3. GigaDevice GD32 standard peripheral firmware
- Path scope: `examples/GD32F303_FOCExplore/software/Firmware/GD32F30x_standard_peripheral/`
- Typical license style in headers: 3-clause BSD-like text
- Example file: `examples/GD32F303_FOCExplore/software/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_adc.c`

4. GigaDevice GD32 USB device library
- Path scope: `examples/GD32F303_FOCExplore/software/Firmware/GD32F30x_usbd_library/`
- Typical license style in headers: 3-clause BSD-like text
- Example file: `examples/GD32F303_FOCExplore/software/Firmware/GD32F30x_usbd_library/usbd/Source/usbd_lld_core.c`

5. GigaDevice GD32 USB FS library
- Path scope: `examples/GD32F303_FOCExplore/software/Firmware/GD32F30x_usbfs_library/`
- Typical license style in headers: 3-clause BSD-like text
- Example file: `examples/GD32F303_FOCExplore/software/Firmware/GD32F30x_usbfs_library/ustd/common/usb_ch9_std.h`

## Compliance Notes

- Preserve original copyright headers and license texts in vendor files.
- Keep this file updated when adding, removing, or replacing third-party code.
- If a component has multiple license variants across subfolders, follow each file header as the final authority.