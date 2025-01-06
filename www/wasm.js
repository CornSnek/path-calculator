let PathfinderWasmObj = null;
let PathfinderExports = null;
let TD = new TextDecoder();
let TE = new TextEncoder();
function JSPrint(BufferAddr, Len, Type) {
  const string = TD.decode(new Uint8Array(PathfinderExports.memory.buffer, BufferAddr, Len));
  postMessage(['f', "JSPrint", Type, string]);
}
function FindPath(query_str, path_algorithm_value) {
  PathfinderExports.WasmFreeAll();
  const enc_grid_str = TE.encode(query_str);
  const alloc_mem = PathfinderExports.WasmAlloc(enc_grid_str.byteLength);
  const mem_view = new Uint8Array(PathfinderExports.memory.buffer, alloc_mem, enc_grid_str.byteLength);
  mem_view.set(enc_grid_str);
  PathfinderExports.FindPath(alloc_mem, enc_grid_str.byteLength, path_algorithm_value);
  PathfinderExports.WasmFree(alloc_mem);
}
function ParsePathfinder(pathfinder_ptr, pathfinder_len, disable_state) {
  PathfinderExports.FlushPrint();
  postMessage(['f', "ParsePathfinder", new Uint32Array(PathfinderExports.memory.buffer, pathfinder_ptr, pathfinder_len)]);
  postMessage(['f', "generate_disable_state", disable_state]);
}
function OutputBruteForcing(str_ptr, str_len) {
  postMessage(['f', "OutputBruteForcing", TD.decode(new Uint8Array(PathfinderExports.memory.buffer, str_ptr, str_len))]);
}
let shared_buffer;
let shared_memory;
let offsets;
function GetOutput() {
  return Atomics.compareExchange(shared_memory, offsets.output, 1, 0);
}
function GetCancel() {
  return Atomics.compareExchange(shared_memory, offsets.cancel, 1, 0);
}
function CalculateBruteForceEarly() {
  return Atomics.compareExchange(shared_memory, offsets.brute_force, 1, 0);
}
WebAssembly.instantiateStreaming(fetch("./pathfinder.wasm"), {
  env: {
    JSPrint,
    ParsePathfinder,
    OutputBruteForcing,
    CalculateBruteForceEarly,
    GetOutput,
    GetCancel,
  },
}).then(result => {
  PathfinderWasmObj = result;
  PathfinderExports = result.instance.exports;
});
const worker_module = {
  FindPath: FindPath,
};
onmessage = onmessage_f;
function onmessage_f(e) {
  if (PathfinderWasmObj != null && PathfinderExports != null) {
    if (e.data[0] == 'f') {
      worker_module[e.data[1]](...e.data.slice(2));
    } else if (e.data[0] = 'm') {
      shared_buffer = e.data[1];
      shared_memory = new Uint8Array(e.data[1]);
      offsets = e.data[2];
    } else {
      console.error("Invalid postMessage flag: " + e.data[0]);
    }
  } else {
    setTimeout(onmessage_f, 1000, e);
  }
}
onerror = e => { console.error(e) };