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
  postMessage(['f', "ParsePathfinder", {s:disable_state, a:new Uint32Array(PathfinderExports.memory.buffer, pathfinder_ptr, pathfinder_len)}]);
  postMessage(['f', "generate_disable_state", disable_state]);
}
function OutputPathfinder(str_ptr, str_len, pn_total_ptr, pn_total_len, pn_now_ptr, pn_now_len) {
  if (pn_total_ptr != 0 && pn_now_ptr != 0) {
    postMessage(['f', "OutputPathfinder", TD.decode(new Uint8Array(PathfinderExports.memory.buffer, str_ptr, str_len)),
      new Uint8Array(PathfinderExports.memory.buffer, pn_total_ptr, pn_total_len),
      new Uint8Array(PathfinderExports.memory.buffer, pn_now_ptr, pn_now_len),
    ]);
  } else {
    postMessage(['f', "OutputPathfinder", TD.decode(new Uint8Array(PathfinderExports.memory.buffer, str_ptr, str_len)), null, null]);
  }
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
function CalculatePathEarly() {
  return Atomics.compareExchange(shared_memory, offsets.brute_force, 1, 0);
}
function MarkColor(str_ptr,str_len,x,y) {
  console.log(str_ptr,str_len)
  postMessage(['f',"MarkColor",TD.decode(new Uint8Array(PathfinderExports.memory.buffer, str_ptr, str_len)),x,y]);
}
function ClearColorGrid() {
  postMessage(['f',"ClearColorGrid"]);
}
WebAssembly.instantiateStreaming(fetch("./pathfinder.wasm"), {
  env: {
    JSPrint,
    ParsePathfinder,
    OutputPathfinder,
    CalculatePathEarly,
    GetOutput,
    GetCancel,
    MarkColor,
    ClearColorGrid,
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