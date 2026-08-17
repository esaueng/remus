# WASM face-evolution contract

## Characterized baseline

The pre-hardening contract is pinned to Remus fork revision
`65e3840c221b20b3d8fd64ca45513d5687c868d6` (the refreshed `origin/main` used
as this work's branch base).

At that revision, `BrepKernel.filletWithEvolution(solid, edges, radius)` was
declared as returning `any` and returned a JavaScript string at runtime. Parsing
that string produced:

```ts
{
  solid: number;
  evolution: {
    modified: Record<string, number[]>;
    generated: Record<string, number[]>;
    deleted: number[];
    unresolved: Record<string, number[]>;
    origin: "construction" | "geometry";
  };
}
```

The numeric keys and values were face arena indices, which are also the public
face handles returned by `getSolidFaces`. `modified` mapped an input face to
the final faces carrying it. `generated` mapped an input face to new final faces
built from it; one blend face normally appeared under both adjacent source
faces. `deleted` named consumed input faces. `unresolved` keyed a final face to
candidate input faces. `solid` was the accepted final solid handle.

That payload did not enumerate the complete input or final face-handle domains.
A decoder therefore could not prove that every input was modified or deleted,
that every claimed output belonged to the final solid, or that an omitted face
was not silently lost. Duplicate and contradictory claims were not rejected.

### Production path and post-processing

The binding snapshotted input face signatures, ran the same fillet engine
cascade used by the production binding, accepted only a closed shell, and then
constructed the evolution map against that accepted result. The rolling-ball
engine assembled a new solid and ran same-surface face unification before
returning it. The binding applied no additional healing or tolerance changes
after the engine returned.

Walking-builder results carried exact construction history. Legacy planar
results did not; the old binding inferred their lineage after final
post-processing from approximate normals and centroids and labeled the map
`origin: "geometry"`.

## Version 1 payload

`filletWithEvolution` and `chamferWithEvolution` now return a typed
`FaceEvolutionPayloadV1` object:

```ts
interface FaceEvolutionPayloadV1 {
  schemaVersion: 1;
  source: { solid: number; faces: number[] };
  result: { solid: number; faces: number[] };
  evolution: {
    provenance: 'construction' | 'unavailable';
    modified: Array<{ source: number; results: number[] }>;
    generated: Array<{ source: number; results: number[] }>;
    deleted: number[];
    unresolvedResults: Array<{ result: number; candidates: number[] }>;
    unresolvedSources: number[];
  };
}
```

`source.faces` is the complete input-face domain captured before the operation.
`result.faces` is the complete face domain of the accepted, post-processed
result. `modified` remains an identity/carry-forward claim. `generated` remains
a construction/adjacency claim and may name multiple source faces for one blend
or bevel face. `deleted` is always present, including when empty.
It contains only inputs that the builder explicitly recorded as absent from the
accepted assembly; absence from `modified` alone is not treated as deletion.

`decodeEvolutionPayload(json)` accepts persisted or transported payloads and
rejects:

- unsupported versions or unknown fields;
- duplicate domain handles, relation entries, or source/result pairs;
- source handles outside `source.faces` or result handles outside
  `result.faces`;
- overlap between modified, generated, and unresolved result claims;
- overlap between modified, deleted, and unresolved source states;
- incomplete source or result coverage; and
- confident claims labeled with `provenance: "unavailable"`.

For a valid payload:

```text
modified sources + deleted + unresolved sources = source.faces
modified results + generated results + unresolved results = result.faces
```

The equalities are set equalities, not counts.

## Provenance and geometry guarantees

The planar fillet and chamfer builders now retain the face associated with each
assembly specification. Assembly refinement carries that association forward,
and rolling-fillet same-surface unification records each pre-unification face's
actual final face. Walking fillet/chamfer builders keep their existing stripe,
trim, and closed-rim construction records.

No stable fillet/chamfer claim is inferred from proximity, traversal position,
or approximate geometric matching. The WASM evolution entry points do not run
the legacy geometry matcher. If an engine cannot provide construction history,
the operation still returns the successful solid but reports every unproven
handle explicitly under `unresolvedSources` or `unresolvedResults` with
`provenance: "unavailable"`.

The evolution entry points share the production geometry engine cascade and
post-processing. Recording history does not change tolerances, healing,
unification, topology validation, failure rollback, or the selected exact
B-Rep. Package smoke tests compare the ordinary and evolution results' binary
solid serialization byte-for-byte.
