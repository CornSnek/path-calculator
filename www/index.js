import { Direction, PathfindingType, PrintType } from './wasm_enums_to_js.js'
let revisited_node_cost;
let default_node_cost;
let error_body;
let error_close;
let error_content;
let generate_grid_button;
let rows_num_input;
let columns_num_input;
let min_random;
let max_random;
let generate_grid_random_button;
let grid_body;
let properties_body;
let properties_description;
let node_x;
let node_y;
let cost_num;
let is_start;
let as_coordinate;
let as_visited;
let path_algorithm;
let path_algorithm_description;
let generate_paths;
let create_paths_body;
let query_save_data;
let query_generate_save;
let generate_brute_force_early;
let cancel_brute_force;
let wasm_worker;
let color1;
let color2;
let color3;
const NPSelect = "Click on a node to select it";
const NPDeselect = "Click on node again or <em class=\"badge\">Esc</em> key to deselect it";
function parse_as_positive_integer() {
  const value = parseInt(this.elem.value);
  if (isNaN(value) || value < 0) {
    set_error_message(`${this.elem_name} must be a positive number.`);
    this.elem.value = 0;
  } else {
    this.elem.value = value;
    hide_error_message();
  }
}
function parse_as_row_column() {
  const value = parseInt(this.elem.value);
  if (isNaN(value) || value <= 0 || value > 15) {
    set_error_message(`${this.elem_name} must be a number from 1 to 15.`);
    this.elem.value = 1;
  } else {
    this.elem.value = value;
    hide_error_message();
  }
}
let event_map = new Map();
document.addEventListener("DOMContentLoaded", init);
async function init() {
  if ('serviceWorker' in navigator) {
    try {
      await navigator.serviceWorker.register('./coi-serviceworker.js', { scope: '/path-calculator/' });
      console.log('COI service worker registered and active');
    } catch (err) {
      console.error('Failed to register COI service worker:', err);
    }
  }
  default_node_cost = document.getElementById("default-node-cost");
  default_node_cost.onchange = parse_as_positive_integer.bind({ elem: default_node_cost, elem_name: "Default Node Cost" });
  revisited_node_cost = document.getElementById("revisited-node-cost");
  revisited_node_cost.onchange = parse_as_positive_integer.bind({ elem: revisited_node_cost, elem_name: "Revisited Node Cost" });
  error_body = document.getElementById("error-body");
  error_close = document.getElementById("error-close");
  error_close.onclick = hide_error_message;
  error_content = document.getElementById("error-content");
  generate_grid_button = document.getElementById("generate-grid");
  rows_num_input = document.getElementById("rows-num");
  rows_num_input.onchange = parse_as_row_column.bind({ elem: rows_num_input, elem_name: "Rows" });
  columns_num_input = document.getElementById("columns-num");
  columns_num_input.onchange = parse_as_row_column.bind({ elem: columns_num_input, elem_name: "Columns" });
  min_random = document.getElementById("min-random");
  min_random.onchange = parse_as_positive_integer.bind({ elem: min_random, elem_name: "Random Min Cost" });
  max_random = document.getElementById("max-random");
  max_random.onchange = parse_as_positive_integer.bind({ elem: max_random, elem_name: "Random Max Cost" });
  generate_grid_random_button = document.getElementById("generate-grid-random");
  grid_body = document.getElementById("grid-body");
  properties_body = document.getElementById("properties-body");
  properties_description = document.getElementById("properties-description");
  properties_description.textContent = NPSelect;
  node_x = document.getElementById("node-x");
  node_y = document.getElementById("node-y");
  cost_num = document.getElementById("cost-num");
  is_start = document.getElementById("is-start");
  as_coordinate = document.getElementById("as-coordinate");
  as_visited = document.getElementById("as-visited");
  path_algorithm = document.getElementById("path-algorithm");
  path_algorithm_description = document.getElementById("path-algorithm-description");
  generate_brute_force_early = document.getElementById("generate-brute-force-early");
  generate_brute_force_early.onclick = () => {
    Atomics.store(shared_memory, offsets.brute_force, 1);
  }
  cancel_brute_force = document.getElementById("cancel-brute-force");
  for (let i = 0; i < PathfindingType.$$length; i++) {
    const option = document.createElement("option");
    path_algorithm.appendChild(option);
    option.value = i;
    option.textContent = PathfindingType.$alt_names[i];
  }
  path_algorithm.onchange = (e) => {
    const pathfinder_int = parseInt(e.target.value);
    path_algorithm_description.textContent = PathfindingType.$description[pathfinder_int];
    if (pathfinder_int !== PathfindingType.brute_forcing) {
      document.querySelectorAll(".brute-force-only").forEach(e => {
        e.style.display = "none";
      });
    } else {
      document.querySelectorAll(".brute-force-only").forEach(e => {
        e.style.display = "initial";
      })
    }

  }
  generate_paths = document.getElementById("generate-paths");
  create_paths_body = document.getElementById("create-paths-body");
  query_save_data = document.getElementById("query-save-data");
  query_generate_save = document.getElementById("query-generate-save");
  query_generate_save.onclick = query_generate_save_f;
  color1 = getComputedStyle(document.body).getPropertyValue("--color1");
  color2 = getComputedStyle(document.body).getPropertyValue("--color2");
  color3 = getComputedStyle(document.body).getPropertyValue("--color3");
  generate_grid_button.onclick = generate_grid;
  generate_grid(null, new URL(window.location.href).searchParams.get("v"));
  generate_grid_random_button.onclick = generate_grid_random;
  generate_paths.onclick = generate_paths_f;
  document.addEventListener("keydown", (event) => {
    if (!event.repeat) {
      const func = event_map.get(event.key);
      if (func !== undefined) {
        func(event);
        update_query_generate();
      }
    }
  });
  function init_shared_buffer(){ //Check crossOriginIsolated before running
    if(window.crossOriginIsolated){
      shared_buffer = new SharedArrayBuffer(3);
      shared_memory = new Uint8Array(shared_buffer);
      create_wasm_worker();
    }else{
      setTimeout(init_shared_buffer,1000);
    }
  }
  init_shared_buffer();
}
function update_query_generate() {
  const link_without_query_str = window.location.href.split("?")[0];
  query_save_data.value = link_without_query_str + "?v=" + encodeURIComponent(generate_query_str());
}
function query_generate_save_f() {
  update_query_generate();
  navigator.clipboard.writeText(query_save_data.value).then(() => alert("Data copied to clipboard")).catch(err => alert(`Error copying text: ${err}`));
  node_f_clear();
}
function set_error_message(err) {
  error_content.innerHTML = err;
  error_body.style.display = "grid";
}
function hide_error_message() {
  error_body.style.display = "none";
}
class Node {
  static None = 0;
  static Start = 1;
  static VisitCoordinate = 2;
  constructor(set_cost) {
    if (set_cost === undefined) {
      const default_nc = parseInt(default_node_cost.value);
      this.cost = !isNaN(default_nc) ? default_nc : 0;
    } else {
      this.cost = set_cost;
    }
    this.type = Node.None;
    this.visited = false;
  }
  get_inner_html() {
    let text = "" + this.cost;
    let do_br = true;
    switch (this.type) {
      case Node.Start:
        if (do_br) {
          text += "<br>";
          do_br = false;
        }
        text += " <em class=\"badge start\">S</em>"
        break;
      case Node.VisitCoordinate:
        if (do_br) {
          text += "<br>";
          do_br = false;
        }
        text += " <em class=\"badge mark\">M</em>"
        break;
    }
    if (this.visited) {
      if (do_br) {
        text += "<br>";
        do_br = true;
      }
      text += " <em class=\"badge visited\">V</em>"
    }
    return text;
  }
}
class Coordinate {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
}
let node_array;
let num_columns;
let num_rows;
let last_div = null;
let start_i = null;
function as_column_row(i) {
  return new Coordinate(i % num_columns, Math.floor(i / num_columns));
}
function node_f_clear() {
  properties_description.textContent = NPSelect;
  node_x.value = "N/A";
  node_y.value = "N/A";
  if (last_div !== null) last_div.style.setProperty("background-color", color1); //Repeated because keybind doesn't remove the color.
  last_div = null;
  event_map.clear();
  cost_num.disabled = true;
  cost_num.onchange = null;
  is_start.disabled = true;
  is_start.onchange = null;
  as_coordinate.disabled = true;
  as_coordinate.onchange = null;
  as_visited.disabled = true;
  as_visited.onchange = null;
}
function node_f() {
  const cr = as_column_row(this.i);
  if (last_div !== null) last_div.style.setProperty("background-color", color1);
  if (last_div !== this.div) {
    properties_description.innerHTML = NPDeselect;
    node_x.value = cr.x;
    node_y.value = cr.y;
    last_div = this.div;
    this.div.style.setProperty("background-color", color2);
    cost_num.disabled = false;
    const this_node = node_array[this.i];
    cost_num.value = this_node.cost;
    cost_num.onchange = () => {
      const new_cost = parseInt(cost_num.value);
      if (!isNaN(new_cost) && new_cost >= 0) {
        this_node.cost = new_cost;
        this.div.innerHTML = this_node.get_inner_html();
        hide_error_message();
      } else {
        set_error_message("Cost must be a positive number.");
        this_node.cost = 0;
        this.div.innerHTML = this_node.get_inner_html();
      }
      cost_num.value = this_node.cost;
    }
    is_start.disabled = false;
    is_start.checked = this_node.type === Node.Start;
    const is_start_onchange_f = () => {
      if (this_node.type === Node.VisitCoordinate) {
        set_error_message("Marked coordinates cannot be the start coordinate.");
        is_start.checked = false;
        return;
      }
      if (start_i !== null && start_i !== this.i) {
        set_error_message("There can only be one start coordinate.");
        is_start.checked = false;
        return;
      }
      this_node.type = is_start.checked ? Node.Start : Node.None;
      this.div.innerHTML = this_node.get_inner_html();
      hide_error_message();
      start_i = is_start.checked ? this.i : null;
    };
    is_start.onchange = is_start_onchange_f;
    event_map.set('s', () => {
      is_start.checked = !is_start.checked;
      is_start_onchange_f();
    });
    as_coordinate.disabled = false;
    as_coordinate.checked = this_node.type === Node.VisitCoordinate;
    const as_coordinate_onchange_f = () => {
      if (this_node.type === Node.Start) {
        set_error_message("Start coordinates cannot be marked to be visited.");
        as_coordinate.checked = false;
      } else {
        this_node.type = as_coordinate.checked ? Node.VisitCoordinate : Node.None;
        this.div.innerHTML = this_node.get_inner_html();
        hide_error_message();
      }
    };
    as_coordinate.onchange = as_coordinate_onchange_f;
    event_map.set('m', () => {
      as_coordinate.checked = !as_coordinate.checked;
      as_coordinate_onchange_f();
    });
    as_visited.disabled = false;
    as_visited.checked = this_node.visited;
    const as_visited_onchange_f = () => {
      this_node.visited = as_visited.checked;
      this.div.innerHTML = this_node.get_inner_html();
    };
    as_visited.onchange = as_visited_onchange_f;
    event_map.set('v', () => {
      as_visited.checked = !as_visited.checked;
      as_visited_onchange_f();
    });
    event_map.set('Escape', node_f_clear);
    event_map.set('c', (e) => {
      e.preventDefault();
      cost_num.select();
    });
    event_map.set('Enter', (e) => {
      e.preventDefault();
      cost_num.blur();
    });
    const divs = Array.from(grid_body.children);
    const right_f = (e) => {
      if (cr.x < num_columns - 1)
        node_f.bind({ i: this.i + 1, div: divs[this.i + 1] })();
      else
        node_f.bind({ i: this.i - num_columns + 1, div: divs[this.i - num_columns + 1] })(); //Wrap if out of bounds.
      e.preventDefault();
    };
    const left_f = (e) => {
      if (cr.x != 0)
        node_f.bind({ i: this.i - 1, div: divs[this.i - 1] })();
      else
        node_f.bind({ i: this.i + num_columns - 1, div: divs[this.i + num_columns - 1] })();
      e.preventDefault();
    };
    const down_f = (e) => {
      if (cr.y < num_rows - 1)
        node_f.bind({ i: this.i + num_columns, div: divs[this.i + num_columns] })();
      else
        node_f.bind({ i: this.i + num_columns - num_columns * num_rows, div: divs[this.i + num_columns - num_columns * num_rows] })();
      e.preventDefault();
    };
    const up_f = (e) => {
      if (cr.y != 0)
        node_f.bind({ i: this.i - num_columns, div: divs[this.i - num_columns] })();
      else
        node_f.bind({ i: this.i + num_columns * num_rows - num_columns, div: divs[this.i + num_columns * num_rows - num_columns] })();
      e.preventDefault();
    };
    event_map.set('ArrowRight', right_f);
    event_map.set('l', right_f);
    event_map.set('ArrowLeft', left_f);
    event_map.set('j', left_f);
    event_map.set('ArrowDown', down_f);
    event_map.set('k', down_f);
    event_map.set('ArrowUp', up_f);
    event_map.set('i', up_f);
  } else {
    node_f_clear();
  }
  update_query_generate();
}
function generate_header_f() {
  generate_paths.disabled = true;
  start_i = null;
  create_paths_body.textContent = "";
  num_columns = parseInt(columns_num_input.value);
  if (isNaN(num_columns) || num_columns == 0 || num_columns > 15) return false;
  num_rows = parseInt(rows_num_input.value);
  if (isNaN(num_rows) || num_rows == 0 || num_rows > 15) return false;
  document.querySelector(":root").style.setProperty("--num-columns", num_columns);
  grid_body.textContent = "";
  node_array = new Array(num_columns * num_rows);
  return true;
}
function generate_grid(_, saved_data) {
  if (saved_data !== null && saved_data !== undefined) { //This branch should just be separated to another function instead (This is called once)
    generate_paths.disabled = true;
    const lines = saved_data.split(/\s*\n/);
    num_columns = null;
    start_i = null;
    num_rows = 0;
    node_array = new Array();
    let node_i = 0;
    let read_state = 0; //0 = revisit or nodes,1 = nodes, 2 = coorinates.
    for (let i = 0; i < lines.length; i++) {
      if (lines[i] === "") continue;
      if (read_state !== 2) {
        if (lines[i] === "c") {
          read_state = 2;
          continue;
        }
        if (read_state === 0) {
          if (lines[i].charAt(0) === "r") {
            const the_int = parseInt(lines[i].substring(1, lines[i].length));
            if (the_int < 0) {
              set_error_message(`Revisited node cost must be positive <br>at line #${i + 1} => \`${lines[i]}\``);
              return;
            }
            revisited_node_cost.value = the_int;
            read_state = 1;
            continue;
          }
        }
        num_rows += 1;
        const node_tokens = lines[i].split(/\s+/);
        if (num_columns === null) {
          num_columns = node_tokens.length;
          document.querySelector(":root").style.setProperty("--num-columns", num_columns);
        } else {
          if (num_columns !== node_tokens.length) {
            set_error_message(`Number of tokens (${node_tokens.length} found) mismatches expected number of columns (${num_columns}) for<br>line #${i + 1} => \`${lines[i]}\``);
            return;
          }
        }
        for (let j = 0; j < node_tokens.length; j++) {
          if (node_tokens[j] === "s") {
            if (start_i !== null) {
              set_error_message(`There cannot be more than one start coordinate.<br>Second start coordinate at line #${i + 1} => \`${lines[i]}\``);
              return;
            }
            node_array.push(new Node());
            node_array[node_i].type = Node.Start;
            node_array[node_i].visited = true;
            start_i = node_i;
          } else {
            const the_int = parseInt(node_tokens[j]);
            if (the_int < 0) {
              set_error_message(`Node costs must be positive <br>at line #${i + 1} => \`${lines[i]}\``);
              return;
            }
            node_array.push(new Node(the_int));
            node_array[node_i].visited = node_tokens[j].endsWith("v");
          }
          const div = document.createElement("div");
          grid_body.appendChild(div);
          div.innerHTML = node_array[node_i].get_inner_html();
          div.onclick = node_f.bind({ i: node_i, div: div });
          node_i += 1;
        }
      } else {
        const coord_tokens = lines[i].split(/\s+/);
        const divs = Array.from(grid_body.children);
        for (let j = 0; j < coord_tokens.length; j++) {
          const comma_i = coord_tokens[j].indexOf(",");
          if (comma_i !== -1) {
            const coord_x = parseInt(coord_tokens[j].substring(0, comma_i));
            const coord_y = parseInt(coord_tokens[j].substring(comma_i + 1, coord_tokens[j].length));
            if (coord_x < 0 || coord_y < 0) {
              set_error_message(`Coordinate format must be 2 positive numbers separated by ','<br>at line #${i + 1} => \`${lines[i]}\``);
              return;
            }
            const coord_i = coord_y * num_columns + coord_x;
            if (start_i === coord_i) {
              set_error_message(`Marked coordinates to visit cannot be a start coordinate<br>at line #${i + 1} => \`${lines[i]}\``);
              return;
            }
            node_array[coord_i].type = Node.VisitCoordinate;
            divs[coord_i].innerHTML = node_array[coord_i].get_inner_html();
          } else {
            set_error_message(`Coordinate format must be 2 positive numbers separated by ','<br>at line #${i + 1} => \`${lines[i]}\``);
            return;
          }
        }
      }
    }
  } else {
    if (!generate_header_f()) return;
    for (let i = 0; i < num_columns * num_rows; i++) {
      node_array[i] = new Node();
      const div = document.createElement("div");
      grid_body.appendChild(div);
      div.textContent = node_array[i].get_inner_html();
      div.onclick = node_f.bind({ i: i, div: div });
    }
  }
  generate_paths.disabled = false;
  update_query_generate();
}
function generate_grid_random() {
  if (!generate_header_f()) return;
  const min_random_num = parseInt(min_random.value);
  const max_random_num = parseInt(max_random.value);
  if (min_random_num > max_random_num) {
    set_error_message("Min Random Cost is greater than Max Random Cost");
    return;
  }
  for (let i = 0; i < num_columns * num_rows; i++) {
    node_array[i] = new Node(Math.floor(Math.random() * (max_random_num - min_random_num + 1)) + min_random_num);
    node_array[i].visited = Math.random() > 0.5;
    const div = document.createElement("div");
    grid_body.appendChild(div);
    div.innerHTML = node_array[i].get_inner_html();
    div.onclick = node_f.bind({ i: i, div: div });
  }
  generate_paths.disabled = false;
  update_query_generate();
}
function generate_query_str() {
  let str_output = `r${revisited_node_cost.value}\n`;
  let coord_arr = new Array();
  for (let y = 0; y < num_rows; y++) {
    for (let x = 0; x < num_columns; x++) {
      const i = y * num_columns + x;
      const node = node_array[i];
      const next_str = i % num_columns != num_columns - 1 ? " " : "\n";
      if (node.type == Node.Start) {
        str_output += `s${next_str}`
      } else {
        str_output += `${node.cost}${node.visited ? "v" : ""}${next_str}`;
        if (node.type == Node.VisitCoordinate) {
          coord_arr.push(new Coordinate(x, y));
        }
      }
    }
  }
  let coord_str = coord_arr.reduce((acc, c) => acc + `${c.x},${c.y} `, "c\n");
  coord_str += "\n";
  return str_output + coord_str;
}
function generate_paths_f() {
  generate_disable_state(true);
  node_f_clear();
  create_paths_body.textContent = "";
  wasm_worker.postMessage(["f", "FindPath", generate_query_str(), path_algorithm.value]);
}
class ColorCoordinate {
  constructor(coord, color) {
    this.coord = coord;
    this.color = color;
  }
}
function highlight_f() {
  const grid_div = Array.from(grid_body.children)[this.y * num_columns + this.x];
  if (this.c !== null) grid_div.style.setProperty("--highlight-color", this.c);
  else grid_div.style.removeProperty("--highlight-color");
  grid_div.classList.add("highlight-coord");
  grid_div.classList.remove("unhighlight-coord");
  this.d.classList.add("highlight-coord");
  this.d.classList.remove("unhighlight-coord");
}
function highlight_all_f() {
  const divs = Array.from(grid_body.children);
  this.node_colors.forEach(nc => {
    const grid_div = divs[nc.coord.y * num_columns + nc.coord.x];
    if (this.c !== null) grid_div.style.setProperty("--highlight-color", nc.color);
    else grid_div.style.removeProperty("--highlight-color");
    grid_div.classList.add("highlight-coord");
    grid_div.classList.remove("unhighlight-coord");
  });
  this.d.classList.add("highlight-coord");
  this.d.classList.remove("unhighlight-coord");
}
function unhighlight_f() {
  const grid_div = Array.from(grid_body.children)[this.y * num_columns + this.x];
  this.d.classList.add("unhighlight-coord");
  this.d.classList.remove("highlight-coord");
  grid_div.classList.add("unhighlight-coord");
  grid_div.classList.remove("highlight-coord");
}
function unhighlight_all_f() {
  const divs = Array.from(grid_body.children);
  this.node_colors.forEach(nc => {
    const grid_div = divs[nc.coord.y * num_columns + nc.coord.x];
    grid_div.classList.add("unhighlight-coord");
    grid_div.classList.remove("highlight-coord");
  });
  this.d.classList.add("unhighlight-coord");
  this.d.classList.remove("highlight-coord");
}
function ParsePathfinder(pathfinder) {
  create_paths_body.textContent = "";
  const node_divs = Array.from(grid_body.children);
  let offset = 2;
  let running_total = 0;
  let path_running_total = 0;
  const total_cost = pathfinder[0];
  const total_paths = pathfinder[1];
  const total_div = document.createElement("div");
  create_paths_body.appendChild(total_div);
  total_div.className = "path-container";
  const total_cost_div = document.createElement("div");
  total_div.appendChild(total_cost_div);
  total_cost_div.classList = "path-coordinate path-cost";
  total_cost_div.innerHTML = `Grand<br>Total<br>${total_cost}`;
  create_paths_body.appendChild(document.createElement("br"));
  let visited_nodes = new Array(num_columns * num_rows);
  for (let i = 0; i < num_columns * num_rows; i++)
    visited_nodes[i] = node_array[i].visited;
  for (let i = 0; i < total_paths; i++) {
    let visited_nodes_clone = Array.from(visited_nodes);
    const paths_div = document.createElement("div");
    create_paths_body.appendChild(paths_div);
    paths_div.className = "path-container";
    const bytes_read = pathfinder[offset];
    const slice = pathfinder.slice(offset + 1, offset + 1 + bytes_read);
    const start_x = slice[0];
    const start_y = slice[1];
    //const end_x = slice[2];
    //const end_y = slice[3];
    const path_cost = slice[4];
    const directions_read = slice[5];
    const path_cost_div = document.createElement("div");
    paths_div.appendChild(path_cost_div);
    path_cost_div.classList = "path-coordinate path-cost";
    path_running_total += path_cost;
    path_cost_div.innerHTML = `Path<br>Total<br>${path_cost}<br><b>${path_running_total}</b>`;
    const highlight_paths_div = document.createElement("div");
    paths_div.appendChild(highlight_paths_div);
    highlight_paths_div.classList = "path-coordinate highlight-path";
    highlight_paths_div.innerHTML = `Highlight Path<br>${i + 1}/${total_paths}`;
    const start_div = document.createElement("div");
    paths_div.appendChild(start_div);
    start_div.classList = "path-coordinate path-start";
    start_div.innerHTML = `${start_x}, ${start_y}<br>Start`;
    start_div.onmouseenter = highlight_f.bind({ x: start_x, y: start_y, d: start_div, c: "green" });
    start_div.onmouseleave = unhighlight_f.bind({ x: start_x, y: start_y, d: start_div });
    start_div.onclick = node_f.bind({ i: start_y * num_columns + start_x, div: node_divs[start_y * num_columns + start_x] });
    let coord_x = start_x;
    let coord_y = start_y;
    const all_node_colors = new Array();
    all_node_colors.push(new ColorCoordinate(new Coordinate(start_x, start_y), "green"));
    highlight_paths_div.onmouseenter = highlight_all_f.bind({ node_colors: all_node_colors, d: highlight_paths_div });
    highlight_paths_div.onmouseleave = unhighlight_all_f.bind({ node_colors: all_node_colors, d: highlight_paths_div });
    for (let d = 0; d < directions_read; d++) {
      const node_cost = slice[6 + 2 * d];
      const direction = slice[6 + 2 * d + 1];
      const direction_name = Direction.$$names[direction];
      switch (direction) {
        case Direction.left: coord_x -= 1; break;
        case Direction.right: coord_x += 1; break;
        case Direction.up: coord_y -= 1; break;
        case Direction.down: coord_y += 1; break;
      }
      const direction_div = document.createElement("div");
      paths_div.appendChild(direction_div);
      if (d != directions_read - 1) {
        direction_div.classList = "path-coordinate path-arrow";
        const node_color = !visited_nodes[coord_y * num_columns + coord_x] ? null : "blue";
        direction_div.onmouseenter = highlight_f.bind({ x: coord_x, y: coord_y, d: direction_div, c: node_color });
        all_node_colors.push(new ColorCoordinate(new Coordinate(coord_x, coord_y), node_color));
        visited_nodes_clone[coord_y * num_columns + coord_x] = true;
      } else {
        direction_div.classList = "path-coordinate path-arrow path-end";
        direction_div.onmouseenter = highlight_f.bind({ x: coord_x, y: coord_y, d: direction_div, c: "red" });
        all_node_colors.push(new ColorCoordinate(new Coordinate(coord_x, coord_y), "red"));
        visited_nodes_clone[coord_y * num_columns + coord_x] = true;
      }
      running_total += node_cost;
      direction_div.innerHTML = `${coord_x}, ${coord_y}<br>${node_cost}${visited_nodes[coord_y * num_columns + coord_x] ? "<em class=\"badge visited\">V</em>" : ""}<br><b>${running_total}</b>`;
      direction_div.onmouseleave = unhighlight_f.bind({ x: coord_x, y: coord_y, d: direction_div });
      direction_div.onclick = node_f.bind({ i: coord_y * num_columns + coord_x, div: node_divs[coord_y * num_columns + coord_x] });
      direction_div.style = `background-image: url(images/${direction_name}.png);`;
    }
    visited_nodes = visited_nodes_clone;
    create_paths_body.appendChild(document.createElement("br"));
    offset += bytes_read + 1;
  }
}

const worker_handler_module = {
  set_error_message,
  generate_disable_state,
  ParsePathfinder,
  OutputBruteForcing,
  JSPrint,
}
let shared_buffer;
let shared_memory;
const offsets = Object.freeze({
  output: 0,
  brute_force: 1,
  cancel: 2,
});
function create_wasm_worker() {
  wasm_worker = new Worker("wasm.js");
  wasm_worker.onmessage = e => {
    if (e.data[0] == 'f') {
      worker_handler_module[e.data[1]](...e.data.slice(2));
    } else {
      console.error("Invalid postMessage flag: " + e.data[0]);
    }
  };
  wasm_worker.onerror = () => generate_disable_state(false);
  wasm_worker.postMessage(['m', shared_buffer, offsets]);
  cancel_brute_force.onclick = () => {
    Atomics.store(shared_memory, offsets.cancel, 1);
  }
}
function OutputBruteForcing(str) {
  path_algorithm_description.innerHTML = str;
}
let output_interval_i = null;
function generate_disable_state(bool) {
  revisited_node_cost.disabled = bool;
  default_node_cost.disabled = bool;
  columns_num_input.disabled = bool;
  rows_num_input.disabled = bool;
  min_random.disabled = bool;
  max_random.disabled = bool;
  generate_grid_button.disabled = bool;
  generate_grid_random_button.disabled = bool;
  generate_paths.disabled = bool;
  path_algorithm.disabled = bool;
  generate_brute_force_early.disabled = !bool;
  cancel_brute_force.disabled = !bool;
  if (bool) {
    if (output_interval_i === null)
      output_interval_i = setInterval(() => Atomics.store(shared_memory, offsets.output, 1), 1000);
  } else {
    if (output_interval_i !== null)
      clearInterval(output_interval_i);
    output_interval_i = null;
  }
}
function JSPrint(Type, string) {
  if (Type === PrintType.log) {
    console.log(string);
  } else if (Type === PrintType.warn) {
    console.warn(string);
  } else {
    set_error_message(string);
    console.error(string);
  }
}