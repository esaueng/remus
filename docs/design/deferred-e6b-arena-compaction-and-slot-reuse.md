# E6b: Arena compaction and slot reuse (deferred design)

Status: deferred. E6a adds permanent retirement tombstones and deliberately
does not reclaim or reuse slots. E6b would change handle lifetime semantics and
therefore requires an explicit compatibility design before implementation.

## Context

Arena IDs are currently raw stable indices. Allocation always appends, and a
retired slot stays dead for the lifetime of that topology. Checkpoint restore
also preserves this no-reuse invariant. As a result, a stale Rust or WASM
handle can never silently refer to a different entity.

Compaction or free-list reuse would break that guarantee unless every handle
can distinguish the old occupant from the new one. Reclaiming Vec capacity,
repacking live entities, and reusing an index are separate operations and
should not be hidden behind one ambiguous delete API.

## Goals

- Offer an explicit way to reclaim memory after many retirements.
- Guarantee that stale handles fail rather than alias new entities.
- Remap every topology reference, pcurve key, and supported root atomically.
- Define interaction with checkpoints, compounds, comp-solids, assemblies,
  attributes, sketches, and WASM handles before changing storage.
- Keep E6a deletion behavior and its no-reuse guarantee unchanged.

This work must not introduce automatic background compaction, silently
invalidate handles during ordinary modeling, or make handle validity depend on
allocation coincidence.

## Design options

### Option A: Copy live roots into a new kernel

Build a new topology by traversing selected live roots, densely remapping the
reachable graph, and return the new kernel plus an explicit root remap. The old
kernel and every old handle remain valid and unchanged until the caller drops
them.

Advantages:

- preserves today's handle invariant within each kernel;
- makes the reclamation boundary obvious;
- can reuse the dense traversal concepts from arena I/O without serializing
  floating-point values;
- is naturally transactional because failure leaves the old kernel untouched.

Costs:

- peak memory temporarily includes both kernels;
- clients must replace their kernel and translate retained roots;
- session state needs an explicit include/exclude policy.

This is the preferred first reclamation feature because it is additive and
does not require slot reuse.

### Option B: In-place compaction with an epoch

Repack all live arenas, update internal references, increment a kernel epoch,
and invalidate every externally held handle. Handles would need to carry or be
checked against that epoch.

This can reclaim memory without two long-lived kernels, but current WASM
handles are plain u32 indices and contain no epoch. A separate kernel-level
epoch check also cannot distinguish individual reused slots after normal
allocation unless all pre-compaction handles are invalidated together.

### Option C: Generational slots with reuse

Store a generation per slot and represent identity as index plus generation.
Retirement increments the generation before a slot can be reused. Lookups
validate both fields.

This supports bounded slot reuse, but it is the most invasive option:

- every typed ID, hash key, topology record, pcurve key, evolution map, and FFI
  conversion changes;
- the current u32 WASM handle may not have enough bits for safe index and
  generation ranges;
- packing generations into u32 introduces wraparound and capacity policy;
- moving to u64 or opaque JS objects is a public API and TypeScript change;
- serialized/debug handle assumptions and downstream consumers need migration.

Generation wrap must fail closed. Reusing a generation value that can still be
held by a client is not acceptable.

## Recommended staged approach

1. Implement additive copy-to-new-kernel compaction first.
2. Measure whether its peak-memory cost is a real production blocker.
3. Consider in-place epochs only if callers can accept whole-kernel handle
   invalidation.
4. Consider generational reuse only as a separately versioned handle API.

No stage should change Arena::alloc, Arena::retire, or restore behavior for
existing callers.

## Proposed copy-compaction contract

The native API should take explicit roots and return both a new topology and
typed remaps:

    compact_roots(topology, solid_roots, compound_roots)
      -> CompactedTopology {
           topology,
           solids,
           compounds,
           entity_remap
         }

Questions to settle before exposing entity_remap:

- Is the remap public for every entity type or only for requested roots?
- Are unreferenced live entities copied, rejected, or deliberately dropped?
- Are duplicate roots preserved as aliases?
- Are comp-solids and assemblies roots in the first release?
- Are pcurves copied only when both owning edge and face are reachable?

The WASM boundary should not mutate an existing BrepKernel in place. An
additive API could return a serialized transfer object for constructing a new
kernel, or a higher-level wrapper could swap kernels and return new root
handles. JavaScript cannot safely retain the old numeric handles after an
in-place swap.

## Complete remapping requirements

Any implementation must walk and rewrite, at minimum:

- solid outer and inner shells;
- shell faces;
- face outer and inner wires;
- oriented wire edges;
- edge endpoints;
- pcurve edge/face keys;
- compound and comp-solid members;
- every selected external root.

Before adding session-state support, separately specify checkpoint snapshots,
GCS sketches, assemblies, evolution maps, and future topology attributes.
Copying only topology while claiming whole-kernel compaction would be
incorrect.

## Failure and checkpoint semantics

Copy compaction should be transactional by construction. In-place compaction
would need a complete staged remap and validation before swapping any arena.

Open checkpoints retain old topology snapshots. An in-place operation must
either reject while checkpoints exist, compact every snapshot, or explicitly
discard them with caller approval. Silent checkpoint invalidation is not
acceptable. Copy-to-new-kernel compaction can leave checkpoints with the old
kernel and start the new kernel with none.

## Validation plan

Before implementation, approve the handle and state-coverage decisions above.
Then require:

- property tests that every reachable reference resolves after remapping;
- stale-handle tests proving no old handle aliases a new entity;
- shared-subtree, inner-shell, pcurve, compound, and comp-solid fixtures;
- checkpoint tests for the chosen policy;
- exact geometry, tolerance, analytic-parameter, and pcurve preservation;
- allocation and memory measurements demonstrating actual reclamation;
- Rust/WASM compatibility tests and an explicit downstream migration plan for
  any new handle representation.

Until that review is complete, permanent tombstones remain the safe public
contract and callers needing memory reclamation should create a new kernel.
