# Gaussian Full Surface / Edges / Corners

The Grafana Gaussian/Foreground panel supports three display modes:

- **Full Surface**: all fitted Gaussian surface samples, within the view budget.
- **Edges** (default): supported geometric creases, curved/linear features, and surface boundaries.
- **Corners**: the room outline estimated from confirmed walls. Cyan solid
  segments have point support. Amber dashed segments show extrapolation and
  any vertical or floor edges without direct support.

The Foreground checkbox and its stream remain independent. These are geometric
features, not semantic object outlines or motion detections. Sparse sampling,
occlusion, holes and measurement noise can cause missing or false edges.

## Data flow and resource limits

Confirmed RoomMap tiles feed the existing Gaussian worker. It reuses the local
covariance fit to classify supported features. One-sided planar neighborhoods
receive an angular coverage check. The classifier excludes isolated samples and
roughly isotropic neighborhoods. Neighboring tile support is included to avoid
turning storage boundaries into visible edges.

At each changed RoomMap checkpoint the Gaussian worker estimates the outline
from its cached planar samples. It aligns the horizontal analysis plane to
Point-LIO gravity when available and uses the current LiDAR position as the
interior reference. A wall needs support along its length and at multiple
heights. Neighboring walls intersect to form candidate corners. The estimate
can remain open when a wall is missing or curved. Three supported intersections
can fill the fourth only if four observed wall planes support a rectangular layout.

The renderer shows projections along gravity when a floor level is available.
Broad floor observations provide a measured level. When a positive Mount Z is
configured, the world origin can serve as an estimated floor datum; the lower
projections are dashed. Set Mount Z to the physical sensor height if the origin
is meant to be on the floor. Without a measured or estimated floor, the panel
shows only the supported upper outline. This uses a level floor model in the
gravity-aligned frame.

Each immutable Gaussian page stores full splats, edge splats and up to 192
planar samples. The outline uses up to 96 nearby tiles per checkpoint and sends
only a bounded set of vertices and segments. Full RoomMap geometry feeds PCD
export and Foreground subtraction.

Edge fitting adds work when tiles change; fewer transmitted/rendered splats do
not guarantee lower total application CPU. Peak RAM, fit time, point counts and
network payload should be measured on the target ADLINK and real scenes.

## Protocol and compatibility

Legacy camera requests remain valid:

```text
VIEW <budget> <height> <camera xyz> <six frustum planes>
```

An optional final `EDGES` or `CORNERS` selects that view on the Gaussian endpoint. `FULL`
explicitly selects the full surface; omitting it preserves the legacy behavior.
Extra tokens and unknown modes are rejected. The ordinary RoomMap endpoint
rejects `EDGES` and `CORNERS` and serves complete RoomMap LOD.

Gaussian metadata includes `supportsEdges`, `supportsRoomLayout` and
`surfaceMode`. In Corners mode the metadata contains versioned `roomLayout`
coordinates and support flags, followed by an empty GSP1 replacement frame.
The outline does not depend on camera motion. Existing radius values in
DeviceConfig files and old CORNERS requests are accepted for compatibility.
Mode changes bypass the camera cooldown. The panel uses the metadata and frame
from the selected mode and map revision.

## Deployment

Build and restart `vista_edge` on ADLINK. Update the existing
`vista-lidarpointcloud-panel` Grafana plugin with version 1.11.0 from the parent
VISTA repository (`dashboard/plugins/lidarpointcloud/dist`), restart Grafana and
hard-refresh the browser. The panel sources are outside this embedded submodule.

Select a mode in the panel toolbar. Save the dashboard to
retain the selection (`options.gaussianSurfaceMode`). Existing V5/V6 dashboards
default to Edges when no mode has been saved. Previously saved Full Surface
choices are preserved; choose Edges and save, or import the updated V5/V6 JSON.
The Foreground checkbox still controls only Foreground.

The panel uses the existing Gaussian WebSocket. It enables each derived view
when the edge application advertises support in its metadata.
