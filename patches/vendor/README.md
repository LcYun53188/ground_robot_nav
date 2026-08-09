# Vendor patch manifest

The top-level repository pins every patched submodule to an upstream commit
that is available from the URL in `.gitmodules`. Local changes are stored only
in this directory and are applied by `scripts/apply_vendor_patches.sh`.

The apply script verifies each exact base commit before changing files. This
prevents a patch from being silently applied to a newer, incompatible upstream
checkout.

| Repository | Upstream base | Patch |
| --- | --- | --- |
| `src/livox_ros_driver2` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | `livox_ros_driver2.patch` |
| `src/FAST_LIO_ROS2` | `2fffc570a25d0df172720bac034fbdb6a13d2162` | `fast_lio_ros2.patch` |
| `third_party/Livox-SDK2` | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` | `livox_sdk2.patch` |
| `src/isaac_ros_nvblox` | `6362295e581ef243773c8a348ac46711e4a1fca4` | `isaac_ros_nvblox.patch` |
| `src/navigation2` | `f3f5d1f64b4905e31ddab3dc5b861f701aa3771c` | `navigation2.patch` |
| `src/magic_enum` | `9f19f78a7d726af84761ecd6d8414613507a95e6` | `magic_enum.patch` |
| `src/isaac_ros_nitros` | `a22f10d4918662c485b0a1323e2fe1d8c21407a9` | `isaac_ros_nitros.patch` |
| `src/negotiated` | `eac198b55dcd052af5988f0f174902913c5f20e7` | `negotiated.patch` |
| `src/isaac_ros_image_pipeline` | `ab21ed0818e50bd4524a442bc186acbde8de8a56` | `isaac_ros_image_pipeline.patch` |
| `src/isaac_ros_nvblox/nvblox_ros/nvblox_core` | `3f42b210df9ad7a2099f00fcf324049d97342cb0` | `nvblox_core.patch` |

`isaac_ros_common`, `isaac_ros_visual_slam`, and FAST-LIO's nested `ikd-Tree`
currently have no local source changes and therefore need no vendor patch.

## Reconstruct the vendor tree

From a clean top-level checkout:

```bash
git submodule update --init --recursive
./scripts/apply_vendor_patches.sh
```

Running the apply script again is safe: it recognizes patches that are already
present. Return to the reproducible upstream bases with:

```bash
./scripts/apply_vendor_patches.sh --reverse
git submodule update --init --recursive --force
```

Use the reverse mode before the forced submodule update because a vendor patch
may add a deliberately ignored file that Git itself will not remove.

## Updating a patch

Build and test a change in its submodule, commit it locally, and regenerate the
patch as the complete difference from the recorded base. Use `--full-index` and
`--binary` so file modes, empty files, and binary changes are retained. Do not
include a nested submodule gitlink in its parent's patch; record the nested
source changes in a separate patch instead.
