# Versioned simulation inputs

| File or folder | Origin and use |
|---|---|
| `tracks/*.mat` | Existing project proxy event tracks; not a claim of exact FSG geometry |
| `tires/Hoosier_10_16_8_LC0_83.mat` | Existing development LC0 fit; does not represent all named R20 tires |
| `reference/r20SlipMapReleaseValidationV6.mat` | Frozen release copied byte-for-byte from the pre-cleanup results on 2026-10-13 |
| `powertrain/A2370DD_T100C.mat` | AMK 100 C source copied byte-for-byte from the local SVN motor-data directory on 2026-10-13 |

Original AMK location:
`C:/Octanes/SVN/VehicleDynamics/05_Sonstiges/AMK_Motordata/A2370DD_Matlab/A2370DD_T100C.mat`.
`lts.powertrain.importAmkMotorMap` now defaults to the project-local copy; an
explicit `Source` can override it. The adapter filters available positive-shaft
points using DC/sqrt(2) and 105 A. It does not recalculate field weakening, model
transient heating or provide measured generator data.

`lts.io.loadR20Reference` loads the frozen released report and rebases only the
vehicle's tire-fit file locations to this checkout. Physical input values and
the frozen binary remain unchanged. Embedded historical provenance may still
contain machine-specific paths; it is not a runtime dependency for current FSG
concept runners.

The original `r20SlipMapBoundaryStudyV5.mat` is unavailable. The optional release
rebuild tool requires a supplied `SourceFile`. The frozen V6 reference supports
current studies but cannot recreate the unavailable historical V5 preparation.

Generated study results belong under `results`, which is ignored by Git. Do not
replace a fixed input with a generated checkpoint without reviewing its identity
and recording a new version/provenance.

## SHA-256 der übernommenen Referenzen

- R20 V6: `b0775e7b4bc2b26c993b18d0bbf3d6cac43587b2d828ecaf4ddaf5d4b1cdf6e3`
- AMK 100 C: `2c862a110937115d3f1dd256a011fc4e3e35cce594f9f89db0a5471858417042`
