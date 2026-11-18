# Gaussian Full Surface / Edges / Corners

The Grafana Gaussian/Foreground panel supports three display modes:

- **Full Surface**: all fitted Gaussian surface samples, within the view budget.
- **Edges** (default): supported geometric creases, curved/linear features, and surface boundaries.
- **Corners**: isolated 3D vertices where three locally observed surfaces meet.

The Foreground checkbox and its stream remain independent. These are geometric
features, not semantic object outlines or motion detections. Sparse sampling,
occlusion, holes and measurement noise can cause missing or false edges.

## Data flow and resource limits

Confirmed RoomMap tiles feed the existing Gaussian worker. It reuses the local
covariance fit to classify supported features. One-sided planar neighborhoods
receive an angular coverage check. The classifier excludes isolated samples and
roughly isotropic neighborhoods. Neighboring tile support is included to avoid
turning storage boundaries into visible edges.

Corners reuse local surface fits to find candidate vertices. They require three
distinct, geometrically independent directions with nearby planar support. The
estimated intersection must lie close to a measured point. The worker selects
the strongest candidate in each neighborhood, including across tile boundaries.
A crease between two faces or a planar outline alone provides too little support.
Occlusions and sparsely observed objects can leave the corner view incomplete.

Each immutable Gaussian page stores its full samples followed by the edge and
corner views. Changed or deleted tiles and their neighbors rebuild on the normal
checkpoint cadence. Each mode reads up to its view budget from the same revision.
Full RoomMap geometry feeds PCD export and Foreground subtraction.

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

Gaussian metadata includes `supportsEdges: true`, `supportsCorners: true` and
`surfaceMode` with value `full_surface`, `edges` or `corners`. GSP1 binary records
remain 44 bytes per splat. Each
client keeps its own mode and camera. Mode changes bypass the same-map camera
cooldown, so frozen maps can switch without waiting for another map revision.
Metadata precedes the replacement frame; the panel ignores frames from a mode
that is no longer selected.

## Deployment

Build and restart `vista_edge` on ADLINK. Update the existing
`vista-lidarpointcloud-panel` Grafana plugin with version 1.5.0 from the parent
VISTA repository (`dashboard/plugins/lidarpointcloud/dist`), restart Grafana and
hard-refresh the browser. The panel sources are outside this embedded submodule.

Select a mode in the panel toolbar. Save the dashboard to
retain the selection (`options.gaussianSurfaceMode`). Existing V5/V6 dashboards
default to Edges when no mode has been saved. Previously saved Full Surface
choices are preserved; choose Edges and save, or import the updated V5/V6 JSON.
The Foreground checkbox still controls only Foreground.

The panel uses the existing Gaussian WebSocket. It enables each derived view
when the edge application advertises support in its metadata.
