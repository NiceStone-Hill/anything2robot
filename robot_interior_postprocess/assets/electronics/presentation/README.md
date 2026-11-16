# CAD appearance references

Prepared on 2026-10-29. These assets improve generation and assembly previews.
They do not redefine collision envelopes, source mounting holes, wire ports,
carrier layout, or printable geometry.

`download_sources.json` records four freshly downloaded native CAD files, their
pinned URLs, SHA-256 hashes and licenses. Every native download was checked
against the existing source conversion provenance before creating the preview.
Original downloads are retained in `source/`.

| Appearance | Primary source | Preview faces | Source parts retained |
| --- | --- | ---: | ---: |
| Arduino UNO R3 | FreeCAD Parts Library, CC BY 3.0 | 68,447 | 782 |
| Pololu D24V50F5 | Pololu official STEP download | 16,000 | 1 |
| 8 x AA battery holder | Adafruit CAD Parts, MIT | 16,000 | 1 |
| SPDT slide switch | Adafruit CAD Parts, MIT | 550 | 1 |

Attribution: [FreeCAD Parts Library](https://github.com/FreeCAD/FreeCAD-library),
[Adafruit CAD Parts](https://github.com/adafruit/Adafruit_CAD_Parts), and
[Pololu D24V50F5](https://www.pololu.com/product/2851/resources).
License texts for the libraries remain in the parent `licenses/` directory.
Pololu provides a public manufacturer download; no open redistribution license
has been confirmed. These files are retained here for local research previews.

The Uno community model omits one legacy LMV358 subobject unsupported by the
original CAD conversion; it is listed in `models.json`, not replaced with a
fabricated component. Battery holder and switch are appearance candidates,
not confirmed installed hardware. All registrations are explicit rigid
transforms at scale 1. The Uno appearance uses the community board outline;
the mounting solution continues to use official Rev3e hole data.

The former process viewer reduced every electronic assembly to 1,200 faces.
For example, the single connected Pololu mesh became 25 islands; many small
Uno parts disappeared. The new preparation preserves every source island and
checks connectivity, Euler characteristic, closedness, dimensions and volume
before accepting simplification. Float64 PLY avoids destroying small faces
during export/reload. Face colors are preserved.

`models.json` records source and preview statistics, registration, source hashes
and omissions. The runtime verifies each preview hash. Native WebGL supplies
depth testing and shading, with an offline Canvas fallback. Components are
not subjected to another global decimation in the process or assembly viewer.

Rebuild previews from the verified native-source/conversion cache:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /home/qinxuan/projects/anything2robot/.conda-env/voxel_tool/bin/python \
  /home/qinxuan/projects/anything2robot/robot_interior_postprocess/prepare_presentation_models.py
```

This explicitly updates display bindings in the cactus assembly configuration.
Subsequent generation is offline. To refresh the native source library, use
the existing `download_models.py` workflow and verify the pinned source hashes.
