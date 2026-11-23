const summary = document.querySelector("#summary");
const notice = document.querySelector("#notice");
const log = document.querySelector("#log");
const mcapInput = document.querySelector("#mcap-path");
const yamlInput = document.querySelector("#yaml-path");
const mapInput = document.querySelector("#map-path");
const assign = document.querySelector("#assign");
const topics = document.querySelector("#topics");
const browser = document.querySelector("#browser");
const playToggle = document.querySelector("#play-toggle");
const scrub = document.querySelector("#scrub");
const clock = document.querySelector("#clock");
const rate = document.querySelector("#rate");

let browseKind = "mcap";
let sourcesFilled = false;
let topicKey = "";
let viewer = PointView.attach(document.querySelector("#view"));
let hidden = new Set();
let lastIndex = -1;
let scrubbing = false;
let remounting = false;
let controlUrl = "http://127.0.0.1:8769";

function parentPath(value) {
  const text = value.trim().replace(/\/+$/, "");
  const slash = text.lastIndexOf("/");
  return slash > 0 ? text.slice(0, slash) : "";
}

function currentPath() {
  if (browseKind === "yaml") {
    return yamlInput.value;
  }
  if (browseKind === "map") {
    return mapInput.value;
  }
  return mcapInput.value;
}

function choosePath(path) {
  if (browseKind === "yaml") {
    yamlInput.value = path;
  } else if (browseKind === "map") {
    mapInput.value = path;
  } else {
    mcapInput.value = path;
  }
  browser.hidden = true;
}

async function listDir(path) {
  const response = await fetch(`/api/browse?kind=${encodeURIComponent(browseKind)}&path=${encodeURIComponent(path)}`);
  const payload = await response.json();
  if (!response.ok) {
    notice.textContent = payload.error || "无法浏览这个目录";
    return;
  }
  browser.hidden = false;
  browser.innerHTML = "";
  const title = document.createElement("p");
  title.textContent = payload.path || "数据目录";
  browser.append(title);
  if (browseKind === "map" && payload.map_dir && payload.path) {
    const useDir = document.createElement("button");
    useDir.type = "button";
    useDir.textContent = "使用此目录作为地图";
    useDir.addEventListener("click", () => choosePath(payload.path));
    browser.append(useDir);
  }
  const up = document.createElement("button");
  up.type = "button";
  up.textContent = payload.path ? "上级目录" : "数据目录";
  up.addEventListener("click", () => listDir(payload.parent || ""));
  browser.append(up);
  for (const entry of payload.entries) {
    const row = document.createElement("button");
    row.type = "button";
    row.textContent = entry.name;
    row.addEventListener("click", () => {
      if (entry.kind === "dir") {
        listDir(entry.path);
        return;
      }
      choosePath(entry.path);
    });
    browser.append(row);
  }
}

function resetView() {
  const old = document.querySelector("#view");
  const canvas = old.cloneNode(false);
  old.replaceWith(canvas);
  viewer = PointView.attach(canvas);
  hidden = new Set();
}

function renderTopics(state) {
  const key = state.topics.map((topic) => `${topic.name}:${topic.suggested}`).join("|");
  assign.hidden = state.topics.length === 0;
  if (!state.topics.length || key === topicKey) {
    return;
  }
  topicKey = key;
  topics.innerHTML = "";
  for (const topic of state.topics) {
    const row = document.createElement("div");
    row.className = "topic";
    const name = document.createElement("div");
    name.className = "name";
    const title = document.createElement("strong");
    title.textContent = topic.name;
    title.title = topic.name;
    const frame = document.createElement("span");
    frame.textContent = topic.frame_id || "没有 frame_id";
    name.append(title, frame);
    const select = document.createElement("select");
    select.dataset.topic = topic.name;
    const empty = document.createElement("option");
    empty.value = "";
    empty.textContent = "不播放";
    select.append(empty);
    for (const item of state.keys) {
      const option = document.createElement("option");
      option.value = item;
      option.textContent = item;
      if (item === topic.suggested) {
        option.selected = true;
      }
      select.append(option);
    }
    row.append(name, select);
    topics.append(row);
  }
}

function applyVisibility() {
  for (const topic of document.querySelectorAll("#topics select")) {
    const color = topic.dataset.color;
    if (color) {
      viewer.setVisible(hexColor(color), !hidden.has(topic.dataset.topic));
    }
  }
}

function hexColor(value) {
  return [
    parseInt(value.slice(1, 3), 16) / 255,
    parseInt(value.slice(3, 5), 16) / 255,
    parseInt(value.slice(5, 7), 16) / 255,
  ];
}

async function refreshView(force) {
  const response = await fetch("/api/view");
  if (!response.ok) {
    return;
  }
  const layers = PointView.parse(await response.arrayBuffer());
  if (!force && !layers.length) {
    return;
  }
  viewer.setLayers(layers);
  for (const item of document.querySelectorAll("#topics select")) {
    const color = item.dataset.color;
    if (color && hidden.has(item.dataset.topic)) {
      viewer.setVisible(hexColor(color), false);
    }
  }
}

async function refresh() {
  const response = await fetch("/api/state");
  const state = await response.json();
  if (state.control_url) {
    controlUrl = state.control_url;
  }
  summary.textContent = state.message;
  notice.textContent = state.error || "";
  log.textContent = state.frame ? `${state.frame}\n${state.message}` : state.message;
  if (!sourcesFilled) {
    mcapInput.value = state.mcap || state.suggest_mcap || "";
    yamlInput.value = state.yaml || state.suggest_yaml || "";
    mapInput.value = state.map || state.suggest_map || "";
    sourcesFilled = true;
  }
  const locked = state.busy;
  mcapInput.disabled = locked;
  yamlInput.disabled = locked;
  mapInput.disabled = locked;
  document.querySelector("#inspect").disabled = locked;
  document.querySelector("#start").disabled = locked || state.phase === "setup" || state.phase === "inspect";
  renderTopics(state);
  for (const select of topics.querySelectorAll("select")) {
    const item = (state.assignments || []).find((entry) => entry.topic === select.dataset.topic);
    if (item) {
      select.dataset.color = item.color;
    }
  }
  if (document.activeElement !== rate) {
    const matched = [...rate.options].find((item) => Number(item.value) === Number(state.rate));
    if (matched) {
      rate.value = matched.value;
    }
  }
  playToggle.disabled = state.frames === 0;
  playToggle.textContent = state.playing ? "暂停" : "播放";
  scrub.disabled = state.frames === 0 || scrubbing;
  scrub.max = Math.max(0, state.frames - 1);
  if (!scrubbing) {
    scrub.value = String(state.index || 0);
  }
  rate.value = String(state.rate);
  clock.textContent = `${state.time.toFixed(1)} / ${state.duration.toFixed(1)} s`;
  if (state.phase === "play" && state.index !== lastIndex) {
    const first = lastIndex < 0;
    lastIndex = state.index;
    if (first) {
      resetView();
    }
    await refreshView(first);
  }
}

document.querySelector("#browse-mcap").addEventListener("click", () => {
  browseKind = "mcap";
  listDir(parentPath(currentPath()));
});
document.querySelector("#browse-yaml").addEventListener("click", () => {
  browseKind = "yaml";
  listDir(parentPath(currentPath()));
});
document.querySelector("#browse-map").addEventListener("click", () => {
  browseKind = "map";
  listDir(parentPath(currentPath()));
});
document.querySelector("#clear-map").addEventListener("click", () => {
  mapInput.value = "";
});

function currentPaths() {
  return {
    mcap: mcapInput.value.trim(),
    yaml: yamlInput.value.trim(),
    map: mapInput.value.trim(),
    paths: [mcapInput.value.trim(), yamlInput.value.trim(), mapInput.value.trim()].filter(Boolean),
  };
}

async function waitForViewer() {
  const deadline = Date.now() + 60000;
  while (Date.now() < deadline) {
    try {
      const response = await fetch("/api/state");
      if (response.ok) {
        return;
      }
    } catch (error) {
      // 容器正在重启。
    }
    await new Promise((resolve) => setTimeout(resolve, 400));
  }
  throw new Error("重新挂载后网页没有起来");
}

async function remountCurrent() {
  remounting = true;
  notice.textContent = "正在按当前路径重新挂载";
  try {
    const response = await fetch(`${controlUrl}/remount`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(currentPaths()),
    });
    const payload = await response.json();
    if (!response.ok) {
      notice.textContent = payload.error || "挂载失败";
      return false;
    }
    if (payload.restarted) {
      await waitForViewer();
      sourcesFilled = false;
      topicKey = "";
      lastIndex = -1;
      notice.textContent = `已挂载 ${payload.added.join("、")}`;
    } else {
      notice.textContent = "这些路径已经在当前挂载里";
    }
    return true;
  } catch (error) {
    notice.textContent = String(error);
    return false;
  } finally {
    remounting = false;
  }
}

async function postJson(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const payload = await response.json();
  return { response, payload };
}

document.querySelector("#remount").addEventListener("click", () => {
  remountCurrent();
});
document.querySelector("#inspect").addEventListener("click", async () => {
  topicKey = "";
  notice.textContent = "";
  const body = { mcap: mcapInput.value.trim(), yaml: yamlInput.value.trim() };
  let { response, payload } = await postJson("/api/inspect", body);
  if (!response.ok && String(payload.error || "").includes("路径不可用")) {
    if (await remountCurrent()) {
      ({ response, payload } = await postJson("/api/inspect", body));
    }
  }
  if (!response.ok) {
    notice.textContent = payload.error || "读取失败";
  }
});
document.querySelector("#start").addEventListener("click", async () => {
  const assignments = [];
  for (const select of topics.querySelectorAll("select")) {
    if (select.value) {
      assignments.push({ topic: select.dataset.topic, key: select.value });
    }
  }
  notice.textContent = "";
  const body = {
    mcap: mcapInput.value.trim(),
    yaml: yamlInput.value.trim(),
    map: mapInput.value.trim(),
    assignments,
  };
  let { response, payload } = await postJson("/api/play", body);
  if (!response.ok && String(payload.error || "").includes("路径不可用")) {
    if (await remountCurrent()) {
      ({ response, payload } = await postJson("/api/play", body));
    }
  }
  if (!response.ok) {
    notice.textContent = payload.error || "无法播放";
    return;
  }
  lastIndex = -1;
});
playToggle.addEventListener("click", async () => {
  const playing = playToggle.textContent !== "暂停";
  await fetch("/api/transport", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ playing }),
  });
});
scrub.addEventListener("pointerdown", () => {
  scrubbing = true;
});
scrub.addEventListener("change", async () => {
  scrubbing = false;
  await fetch("/api/transport", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ index: Number(scrub.value), playing: false }),
  });
});
rate.addEventListener("change", async () => {
  await fetch("/api/transport", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ rate: Number(rate.value) }),
  });
});

storedSize("--side-width", "560px");
storedSize("--output-height", "220px");
dragSize(document.querySelector("#side-resizer"), () => document.querySelector("#side").getBoundingClientRect().width, (origin, dx) => {
  const width = Math.max(420, Math.min(980, origin + dx));
  document.documentElement.style.setProperty("--side-width", `${width}px`);
  localStorage.setItem("rtk-side-width", `${width}px`);
});

function storedSize(name, fallback) {
  const key = name === "--side-width" ? "rtk-side-width" : "rtk-output-height";
  const saved = localStorage.getItem(key);
  document.documentElement.style.setProperty(name, saved || fallback);
}

function dragSize(handle, read, write) {
  handle.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    const startX = event.clientX;
    const startY = event.clientY;
    const origin = read();
    function move(ev) {
      write(origin, ev.clientX - startX, ev.clientY - startY);
    }
    function stop() {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", stop);
    }
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", stop);
  });
}

setInterval(() => {
  if (remounting) {
    return;
  }
  refresh().catch((error) => {
    notice.textContent = String(error);
  });
}, 200);
refresh().catch((error) => {
  notice.textContent = String(error);
});
