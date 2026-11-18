# Gaussian Full Surface / Edges

The Grafana Gaussian/Foreground panel supports two display modes:

- **Full Surface**: all fitted Gaussian surface samples, within the view budget.
- **Edges** (default): supported geometric creases, curved/linear features, and surface boundaries.

The Foreground checkbox and its stream remain independent. These are geometric
features, not semantic object outlines or motion detections. Sparse sampling,
occlusion, holes and measurement noise can cause missing or false edges.

## Data flow and resource limits

Confirmed RoomMap tiles feed the existing Gaussian worker. It reuses the local
covariance fit to classify supported features. One-sided planar neighborhoods
receive an angular coverage check. The classifier excludes isolated samples and
roughly isotropic neighborhoods. Neighboring tile support is included to avoid
turning storage boundaries into visible edges.

Each immutable Gaussian page stores its full samples followed by an edge subset.
This consumes extra temporary disk space for the subset, but allows either mode
to perform budget-limited random reads. Camera changes and mode changes do not
repeat feature extraction. Changed/deleted tiles and their neighbors rebuild
using the existing checkpoint cadence. Both views share revision lifetimes and
cache cleanup. Full RoomMap geometry, exported PCD, localization and Foreground
background subtraction remain unchanged.

Edge fitting adds work when tiles change; fewer transmitted/rendered splats do
not guarantee lower total application CPU. Peak RAM, fit time, point counts and
network payload should be measured on the target ADLINK and real scenes.

## Protocol and compatibility

Legacy camera requests remain valid:

```text
VIEW <budget> <height> <camera xyz> <six frustum planes>
```

An optional final `EDGES` selects edge splats on the Gaussian endpoint. `FULL`
explicitly selects the full surface; omitting it preserves the legacy behavior.
Extra tokens and unknown modes are rejected. The ordinary RoomMap endpoint
rejects `EDGES` and continues serving complete RoomMap LOD.

Gaussian metadata includes `supportsEdges: true` and `surfaceMode` with value
`full_surface` or `edges`. GSP1 binary records remain 44 bytes per splat. Each
client keeps its own mode and camera. Mode changes bypass the same-map camera
cooldown, so frozen maps can switch without waiting for another map revision.
Metadata precedes the replacement frame; the panel ignores frames from a mode
that is no longer selected.

## Deployment

Build and restart `vista_edge` on ADLINK. Update the existing
`vista-lidarpointcloud-panel` Grafana plugin with version 1.4.1 from the parent
VISTA repository (`dashboard/plugins/lidarpointcloud/dist`), restart Grafana and
hard-refresh the browser. The panel sources are outside this embedded submodule.

Select **Full Surface** or **Edges** in the panel toolbar. Save the dashboard to
retain the selection (`options.gaussianSurfaceMode`). Existing V5/V6 dashboards
default to Edges when no mode has been saved. Previously saved Full Surface
choices are preserved; choose Edges and save, or import the updated V5/V6 JSON.
The Foreground checkbox still controls only Foreground.

No new DeviceConfig setting, worker, port or data-source plugin is required.
An older edge application serves Full Surface; the panel disables the Edges
option after receiving metadata without edge support.
