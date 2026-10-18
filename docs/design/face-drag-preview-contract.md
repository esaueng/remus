# Face-drag preview contract

Owner: P-Class 6.2 and O4.7. Tier: core, because face moves allocate and
validate geometry in the live kernel arena. This adds a typed boundary to the
existing exact operation; it does not expand its supported geometry families.

An application captures the source solid and selected face handles when a drag
starts. Every requested signed distance is measured from that source, rather
than accumulated on the previous preview. `moveFacesDetailed` returns the same
exact result as `moveFaces`, or a typed refusal with no result handle. Direct
and batch calls share the implementation, input/work limits, and rollback.

Only a successful result replaces the displayed geometry and its remembered
distance. A refusal preserves the source and every earlier accepted result,
including their handles, exact arena documents and tessellation. The caller
keeps displaying the last accepted frame, associates the diagnostic with the
requested distance, and may request another distance without restarting the
drag. No approximate mesh is substituted for an invalid solid.

```js
// Capture sourceSolid and sourceFaces once when the gesture starts.
let displayedSolid = sourceSolid;
function preview(distance) {
  const result = kernel.moveFacesDetailed(sourceSolid, sourceFaces, distance);
  if (result.status === 'ok') {
    displayedSolid = result.value;
    presentExactSolid(displayedSolid);
    clearDragDiagnostic();
  } else {
    showDragDiagnostic(distance, result.code, result.details.message);
    // Keep the already displayed solid; no restore or mesh replacement.
  }
}
```

The application coalesces pointer events, rejects responses from obsolete
gestures, and displays the dimension separately from a pending or refused
preview. Release validates the requested edit before changing document history.
If final validation fails, the last accepted preview remains disposable and
visible; keeping it requires an explicit validated commit. Cancel restores the
committed source. A preview must not survive a source revision or selection
change. A successful new preview clears the previous value's diagnostic.

The kernel supplies exact geometry and typed outcomes; pointer handling,
rendering, asynchronous response ownership, and undo history remain consumer
responsibilities. Source tests cover accepted/refused/recovered sequences,
source-relative distances, preserved bytes/meshes, and direct/batch parity.
Installed-package and browser evidence must be reported separately from native
source tests.
