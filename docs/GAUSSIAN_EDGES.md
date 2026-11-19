# Gaussian Full Surface / Edges / Corners

The Grafana Gaussian/Foreground panel offers two surface modes:

- **Full Surface**: all fitted Gaussian surface samples, within the view budget.
- **Edges** (default): supported geometric creases, curved/linear features, and surface boundaries.

The **Corners** checkbox overlays the room outline estimated from confirmed walls.
Cyan solid segments have point support. Amber dashed segments show extrapolation
and vertical or floor edges without direct support.

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

Gaussian metadata includes `supportsEdges`, `supportsRoomLayoutOverlay` and
`surfaceMode`. On the same Gaussian WebSocket, `ROOM LAYOUT ON` subscribes to
the current and subsequent `room-layout` messages. `ROOM LAYOUT OFF` stops
updates for that viewer. The separate messages carry a map generation and a
versioned outline. Toggling the overlay does not request new splats. The older
`CORNERS` view and its optional radius remain accepted for archived plugins.

## Deployment

Build and restart `vista_edge` on ADLINK. Update the existing
`vista-lidarpointcloud-panel` Grafana plugin with version 1.12.0 from the parent
VISTA repository (`dashboard/plugins/lidarpointcloud/dist`), restart Grafana and
hard-refresh the browser. The panel sources are outside this embedded submodule.

Select Edges or Full Surface in the panel toolbar and check Corners to show
the outline. Save the dashboard to keep the selections. Existing V5/V6 dashboards
default to Edges when no mode has been saved. A saved legacy Corners mode appears
as Edges with Corners checked. The Foreground checkbox remains independent.
