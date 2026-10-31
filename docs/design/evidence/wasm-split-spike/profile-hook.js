// Appended to the instrumented Node glue: on exit, dump the wasm-split profile.
if (process.env.WASM_SPLIT_PROFILE_DIR) {
  process.on('exit', () => {
    try {
      const size = 1 << 20;
      const ptr = wasm.__wbindgen_malloc(size, 8);
      const len = wasm.__write_profile(ptr, size);
      const bytes = new Uint8Array(wasm.memory.buffer, ptr, len);
      require('fs').writeFileSync(
        `${process.env.WASM_SPLIT_PROFILE_DIR}/${process.pid}-${Date.now()}.prof`,
        bytes,
      );
    } catch (e) {
      process.stderr.write(`wasm-split profile dump failed: ${e}\n`);
    }
  });
}
