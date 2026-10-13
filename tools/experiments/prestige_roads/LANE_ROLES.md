# Prestige Roads: physical lane roles and bus contraflow

2026-10-10 follow-up to the [five-city stress test](CITIES.md). The current five-city exports contain
**121 layouts, 540 private-car lane connections, 424 marked crossings, 171 inferred stop bars,
and 11 inferred BUS pavement words**. Physical bus lanes retain their space and source lane numbers.
The lane graph in this pass represents private passenger cars authorized under the retained access conditions;
bus movements through intersections and bus signal phases remain unresolved.

![Pittsburgh: source-backed bus contraflow space with inferred BUS words](output/pittsburgh/roles-bus-contraflow.jpg)

## Current exports

| Scene | Layouts | Car connections | Crossings | Stop bars | BUS words |
| --- | ---: | ---: | ---: | ---: | ---: |
| Boston | 26 | 114 | 60 | 48 | 0 |
| Pittsburgh | 14 | 81 | 95 | 43 | 2 |
| London | 32 | 140 | 57 | 35 | 1 |
| Paris | 33 | 121 | 165 | 36 | 8 |
| Sion | 16 | 84 | 47 | 9 | 0 |

The previous five-city checkpoint had 105 layouts and 480 connections. Counts alone are not a correctness
measure: this pass also withdraws an earlier London layout that over-assigned car lanes.

## Physical lanes and vehicle eligibility

`lane_roles.py` separates the physical directional section from the vehicle profile. A three-lane section
keeps all three lane centers and its full width even when only physical lanes 2 and 3 can receive car
connectors. Excluded lanes are not compacted away or silently renumbered. Exported ports contain the
physical roles and eligible lane numbers; the standard layout audit rejects a connector that uses an
excluded lane. Coverage is explicitly scoped to eligible car lanes.

Per-lane access follows the access / vehicle / motor_vehicle / motorcar hierarchy. Empty entries inherit
rather than implying a ban. A source condition such as private or destination access stays in the export;
accepting geometry does not make that road a public route. Equivalent geometric eligibility can continue
through a public/private tag boundary, while source-chain way IDs preserve each way's access evidence.
[OSM lane-access model](https://wiki.openstreetmap.org/wiki/Lanes)

A designated bus or PSV lane alone does not prove that cars are forbidden. Explicit car permissions and
exclusions are retained separately. Bus-specific access overrides inherited PSV access, including when
that means a BUS word must be withheld. Legacy `busway:left/right` tags provide a bounded lateral placement
inference. Without explicit per-lane car permission, their car eligibility is recorded as **unknown** and
withheld from this representative graph; this is not exported as a legal car ban.
[Bus lane access](https://wiki.openstreetmap.org/wiki/Key:bus:lanes),
[PSV inheritance](https://wiki.openstreetmap.org/wiki/Key:psv:lanes),
[legacy busway evidence](https://wiki.openstreetmap.org/wiki/Key:busway)

A car one-way with a supported bus exception can have two physical travel directions. When the source
establishes directional counts, the bus contraflow gets an opposing section and is excluded from the car
profile. An unpositioned reserved-lane count does not justify guessing which lane it occupies. Conditional
rules, unexplained counts, opposite-only bus policies, and contradictory vectors remain explicit failures.
[Physical and reserved lane counts](https://wiki.openstreetmap.org/wiki/Key:lanes:psv)

## Markings and examples

The compiler generates conventional vector BUS words in source-designated incoming lanes. Word presence,
font, size, and position are inferred: 4.5 m long and at most 2.4 m wide. They do not assert bus-only use.
Placement avoids mapped crossing passages, inferred stop-bar reservations, source-indicated arrows, and
neighboring approach handoffs. Unsupported or crowded placements retain omission reasons. `lane_markings=no`
continues to suppress these markings. Glyph holes survive tessellation.

- **Pittsburgh:** Sixth Avenue / Smithfield Street (`104572618`) now preserves opposing bus-lane space,
  two inferred BUS words, and separate approach bars and crosswalks. Oliver Avenue / Smithfield also resolves.
- **Paris:** nine additional layouts include Boulevard Henri IV and the Beaumarchais / Pas de la Mule area.
  Both normal and contraflow designated lanes retain their physical placement.
- **Boston:** Essex Street at Edinboro and Ping On resolves with source-based reserved-lane space.
  These short approaches do not have enough clear room for BUS words; the compiler leaves them unpainted.
- **Sion:** access-restricted physical lane space remains distinct from the car graph. A restriction on
  motor vehicles alone does not justify labeling the lane as a bus lane.

![Paris: designated lane space, inferred labels, and preserved crossings](output/paris/roles-reserved-lanes.jpg)

London's New Oxford Street / Bury Place case (`25472769`) is now deliberately unresolved. Source way
`4393564` has two total lanes, car one-way travel, and a bus exception, but no physical directional split.
The earlier layout assigned both lanes to cars. The source does not support that assumption. Separately,
Lincoln's Inn Fields / Serle Street remains supported after correcting the overly strict access-boundary
chaining rule; private access itself does not create a new physical lane boundary.

## Stop controls on physical contraflow lanes

A subsequent control pass fixes a remaining car-only assumption in fallback stop-bar generation. A signal
facing an established bus contraflow was rejected using the car one-way flag before checking the physical
section. The compiler now validates physical directional counts first, then places the bar across that
section. The source signal direction and inferred paint geometry remain separate; bus phases and turning
movements are not supplied by this change.
[OSM directional signal evidence](https://wiki.openstreetmap.org/wiki/Key:traffic_signals:direction)

This recovers two bars on London's Bloomsbury Way (signal nodes `4356620357` and `4356620367`) and one on
Paris's Boulevard Henri IV (`13495680089`). A new `contraflow-control` camera selects a rendered example
from source evidence, without a hard-coded junction table.

Pittsburgh also exposes ambiguous combined signal/crossing evidence. Fallback bars at nodes `5312909754`,
`5312909851`, and `5312909854` are withdrawn because placing paint at that signal position could put it in
a crossing. Two additional contraflow controls (`5312909756`, `8484461664`) now retain that placement
reason instead of the incorrect car-direction rejection. Dedicated resolution of combined crossing and
stop positions remains unfinished. Existing crossing geometry is unchanged.
[Combined and separately mapped crossings](https://wiki.openstreetmap.org/wiki/Traffic_light)

The total remains **171 bars**: London moves from 33 to 35, Paris from 35 to 36, and Pittsburgh from 46 to 43.
All surviving stop bars keep their prior coordinates; all 121 city layouts and all 29 Manhattan lane-path
sets remain identical to the role checkpoint. Synthetic tests cover both digitization directions, both
traffic sides, missing physical lanes, crossing exclusions, and ambiguous signal/crossing positions.
The [control regressions](output/contraflow-control-tests.json) and
[compiled comparison](output/contraflow-control-export-audit.json) retain source IDs and reasons.

![London: recovered stop bar in a physical bus contraflow section](output/london/roles-contraflow-control.jpg)

## Checks and remaining work

[Role tests](output/lane-role-tests.json) cover hierarchy and inheritance, physical numbering, both traffic
sides, bus contraflow, legacy access uncertainty, malformed and contradictory data, and exclusion of reserved
lanes from car connections. [Label tests](output/bus-label-tests.json) exercise glyph holes and control
exclusions. [Export audits](output/lane-role-export-audit.json) independently check serialized ports,
connectors, source numbering, and label evidence. The city glTF/boundary audits and existing Manhattan and
Alewife regressions also pass; tests remain transient outside the repository.

The [baseline comparison](output/lane-role-regression-comparison.json) confirms exact lane-path preservation
for all 104 surviving first-sweep city layouts and all 29 Manhattan layouts. Seventeen layouts are added;
the London case described above is the sole withdrawal. Alewife remains at six layouts and 31 connections,
with the 180 m three-to-two taper and both measured deck clearances unchanged (5.248 m and 5.292 m).
Representative role layouts in all five cities were inspected directly in osgEarth; the
[viewer record](output/city-viewer-audit.json) distinguishes these captures from earlier surface checks.

This remains a bounded rendering experiment. It does not infer bus turn paths, legal bus-only use,
conditional schedules, traffic priority, bus swept envelopes, or signal phases. Multi-lane compound
junctions and longitudinal turn-pocket entry remain separate work. Heights remain synthetic; the previous
surface-fit and bridge-clearance limitations still apply.

## Inspect

Recompile with `compile.py --scene <city>`. Then use the existing installed runtime:

```powershell
tools/experiments/prestige_roads/run.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo `
    -Scene pittsburgh -View reserved-lanes -SSE 8
```

The automatic `reserved-lanes` view prefers a case with visible inferred labels. Paris and London have
this view as well; use `reserved-lanes-street` for a close approach when listed in the current viewpoint file.
Use `-Scene london -View contraflow-control` or `-Scene paris -View contraflow-control` to inspect the
recovered directional stop bars.
