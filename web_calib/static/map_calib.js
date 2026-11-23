const summary = document.querySelector("#summary");
const notice = document.querySelector("#notice");
const picker = document.querySelector("#picker");
const topics = document.querySelector("#topics");
const cards = document.querySelector("#cards");
const starts = document.querySelector("#starts");
const startsNote = document.querySelector("#starts-note");
const saveStartsButton = document.querySelector("#save-starts");
const restoreStartsButton = document.querySelector("#restore-starts");
const log = document.querySelector("#log");
const output = document.querySelector("#output");
const showResultButton = document.querySelector("#show-result");
const resultDialog = document.querySelector("#result-dialog");
const resultWindow = document.querySelector("#result-window");
const resultBody = document.querySelector("#result-body");
const copyResultButton = document.querySelector("#copy-result");
const closeResultButton = document.querySelector("#close-result");
const hint = document.querySelector("#view-hint");
const canvas = document.querySelector("#view");
const alignButton = document.querySelector("#align");
const optimizeButton = document.querySelector("#optimize");
const resetButton = document.querySelector("#reset");
const changeButton = document.querySelector("#change-data");
const sources = document.querySelector("#sources");
const mapInput = document.querySelector("#map-path");
const mcapInput = document.querySelector("#mcap-path");
const yamlInput = document.querySelector("#yaml-path");
const yamlReview = document.querySelector("#yaml-review");
const loadButton = document.querySelector("#load-data");
const browser = document.querySelector("#browser");
const fields = ["x", "y", "z", "roll", "pitch", "yaw"];
const hiddenClouds = new Set();
const angleFields = new Set(["roll", "pitch", "yaw"]);
let resultKey = "";
let resultText = "";
let resultPath = "";
let browseKind = "";
let sourcesFilled = false;
let built = "";
let cardKey = "";
let shownView = -1;
let viewBusy = false;
let pickingTopic = "";
let editHold = 0;
const livePose = new Map();
const queuedPose = new Map();
const sendingPose = new Set();
const poseEpoch = new Map();
const poseTimers = new Map();
const viewer = PointView.attach(canvas);
viewer.setPickHandler((point) => {
  if (!pickingTopic) {
    return;
  }
  const topic = pickingTopic;
  const card = cards.querySelector(`article[data-topic="${cssEscape(topic)}"]`);
  const pose = card ? readPose(card) : null;
  const next = {
    x: point[0],
    y: point[1],
    z: point[2],
    roll: pose ? pose.roll : 0,
    pitch: pose ? pose.pitch : 0,
    yaw: pose ? pose.yaw : 0,
  };
  clearTimeout(poseTimers.get(topic));
  poseEpoch.set(topic, (poseEpoch.get(topic) || 0) + 1);
  queuedPose.delete(topic);
  if (card && pose) {
    for (const field of ["x", "y", "z"]) {
      const input = card.querySelector(`input[data-field="${field}"]`);
      if (input) {
        input.value = Number(next[field]).toFixed(3);
      }
    }
    livePose.set(topic, next);
    editHold = Date.now() + 400;
    viewer.placeStart(colorOf(card.dataset.color), next);
  } else {
    livePose.delete(topic);
  }
  post("/api/pose", { topic, ...next, keep_rotation: Boolean(pose) });
});

function cssEscape(value) {
  return window.CSS && CSS.escape ? CSS.escape(value) : value.replace(/"/g, "");
}

function selectedTopics() {
  return [...topics.querySelectorAll("input[data-topic]:checked")].map((input) => input.dataset.topic);
}

function referenceTopic() {
  const chosen = topics.querySelector("input[name='reference']:checked");
  return chosen ? chosen.value : "";
}

function extrinsicChoices() {
  const chosen = {};
  for (const select of topics.querySelectorAll("select.extrinsic")) {
    if (select.value) {
      chosen[select.dataset.topic] = select.value;
    }
  }
  return chosen;
}

async function post(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  const payload = await response.json();
  if (!response.ok) {
    notice.textContent = payload.error || "请求失败";
  }
  return payload;
}

alignButton.addEventListener("click", () => {
  const chosen = selectedTopics();
  const reference = referenceTopic();
  if (!chosen.includes(reference)) {
    notice.textContent = "基准雷达必须在勾选的雷达里";
    return;
  }
  notice.textContent = "";
  pickingTopic = "";
  post("/api/align", { topics: chosen, reference, extrinsics: extrinsicChoices() });
});
optimizeButton.addEventListener("click", () => post("/api/optimize"));
resetButton.addEventListener("click", () => {
  built = "";
  pickingTopic = "";
  post("/api/reset");
});
changeButton.addEventListener("click", () => {
  sourcesFilled = false;
  pickingTopic = "";
  browser.hidden = true;
  post("/api/sources");
});
saveStartsButton.addEventListener("click", () => post("/api/save-starts"));
restoreStartsButton.addEventListener("click", () => post("/api/restore-starts"));
loadButton.addEventListener("click", () => {
  notice.textContent = "";
  post("/api/load", {
    map: mapInput.value.trim(),
    mcap: mcapInput.value.trim(),
    yaml: yamlInput.value.trim(),
  });
});
document.querySelector("#match-extrinsic").addEventListener("click", () => {
  built = "";
  notice.textContent = "";
  post("/api/extrinsics", { yaml: yamlReview.value.trim() });
});
document.querySelector("#browse-map").addEventListener("click", () => openBrowser("map"));
document.querySelector("#browse-mcap").addEventListener("click", () => openBrowser("mcap"));
document.querySelector("#browse-yaml").addEventListener("click", () => openBrowser("yaml"));

function parentPath(value) {
  const text = value.trim().replace(/\/+$/, "");
  const slash = text.lastIndexOf("/");
  return slash > 0 ? text.slice(0, slash) : "";
}

async function openBrowser(kind) {
  browseKind = kind;
  const current = kind === "map" ? mapInput.value : kind === "yaml" ? yamlInput.value : mcapInput.value;
  await listDir(parentPath(current));
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

function choosePath(path) {
  if (browseKind === "map") {
    mapInput.value = path;
  } else if (browseKind === "yaml") {
    yamlInput.value = path;
    yamlReview.value = path;
  } else {
    mcapInput.value = path;
  }
  browser.hidden = true;
}

function renderSources(state) {
  const choosing = state.phase === "sources" || state.phase === "loading";
  sources.hidden = !choosing;
  if (choosing && !sourcesFilled) {
    mapInput.value = state.map_path || state.suggest_map || "";
    mcapInput.value = state.mcap || state.suggest_mcap || "";
    yamlInput.value = state.extrinsic_path || state.suggest_yaml || "";
    sourcesFilled = true;
  }
  const locked = state.busy || state.phase === "loading";
  mapInput.disabled = locked;
  mcapInput.disabled = locked;
  yamlInput.disabled = locked;
  loadButton.disabled = locked;
  document.querySelector("#browse-map").disabled = locked;
  document.querySelector("#browse-mcap").disabled = locked;
  document.querySelector("#browse-yaml").disabled = locked;
}

function renderTopics(state) {
  const keys = state.extrinsic_keys || [];
  const key = state.topics.map((topic) => `${topic.name}:${topic.extrinsic || ""}`).join("|")
    + state.reference + keys.join(",");
  if (document.activeElement !== yamlReview && state.phase === "select") {
    yamlReview.value = state.extrinsic_path || yamlInput.value || state.suggest_yaml || "";
  }
  document.querySelector("#match-extrinsic").disabled = state.busy;
  if (built === key) {
    return;
  }
  built = key;
  topics.innerHTML = "";
  for (const topic of state.topics) {
    const row = document.createElement("div");
    row.className = "topic";
    const useLabel = document.createElement("label");
    const use = document.createElement("input");
    use.type = "checkbox";
    use.dataset.topic = topic.name;
    use.checked = true;
    useLabel.append(use, document.createTextNode("标定"));
    const refLabel = document.createElement("label");
    const ref = document.createElement("input");
    ref.type = "radio";
    ref.name = "reference";
    ref.value = topic.name;
    ref.checked = topic.name === state.reference;
    refLabel.append(ref, document.createTextNode("基准"));
    const text = document.createElement("span");
    const frame = topic.frame_id ? `  ${topic.frame_id}` : "";
    text.textContent = `${topic.label}${frame}  ${topic.name}`;
    row.append(useLabel, refLabel, text);
    if (keys.length) {
      const select = document.createElement("select");
      select.className = "extrinsic";
      select.dataset.topic = topic.name;
      const empty = document.createElement("option");
      empty.value = "";
      empty.textContent = "不使用";
      select.append(empty);
      for (const name of keys) {
        const option = document.createElement("option");
        option.value = name;
        option.textContent = name;
        if (name === topic.extrinsic) {
          option.selected = true;
        }
        select.append(option);
      }
      row.append(select);
    }
    topics.append(row);
  }
}

function isAngle(field) {
  return angleFields.has(field);
}

function toDegrees(radians) {
  return Number(radians) * 180 / Math.PI;
}

function toRadians(degrees) {
  return Number(degrees) * Math.PI / 180;
}

function readPose(card) {
  const pose = {};
  for (const field of fields) {
    const input = card.querySelector(`input[data-field="${field}"]`);
    if (!input || input.value.trim() === "" || Number.isNaN(Number(input.value))) {
      return null;
    }
    pose[field] = isAngle(field) ? toRadians(input.value) : Number(input.value);
  }
  return pose;
}

function poseClose(left, right) {
  if (!left || !right) {
    return false;
  }
  const position = Math.hypot(left.x - right.x, left.y - right.y, left.z - right.z);
  const angle = Math.abs(left.roll - right.roll) + Math.abs(left.pitch - right.pitch) + Math.abs(left.yaw - right.yaw);
  return position < 0.002 && angle < 0.0005;
}

function colorOf(hex) {
  const text = String(hex).replace("#", "");
  return [0, 2, 4].map((index) => parseInt(text.slice(index, index + 2), 16) / 255);
}

function fillPose(card, pose, topic) {
  const live = livePose.get(topic);
  const shown = live && pose && !poseClose(live, pose) ? live : pose;
  if (live && pose && poseClose(live, pose)) {
    livePose.delete(topic);
  }
  if (!shown) {
    return;
  }
  for (const field of fields) {
    const input = card.querySelector(`input[data-field="${field}"]`);
    if (!input || document.activeElement === input) {
      continue;
    }
    const digits = isAngle(field) ? 3 : 3;
    const value = isAngle(field) ? toDegrees(shown[field]) : Number(shown[field]);
    const text = value.toFixed(digits);
    if (input.value !== text) {
      input.value = text;
    }
  }
}

function schedulePose(topic, card) {
  clearTimeout(poseTimers.get(topic));
  poseTimers.set(topic, setTimeout(() => pushPose(topic, card), 40));
}

function pushPose(topic, card) {
  const pose = readPose(card);
  if (!pose) {
    return;
  }
  livePose.set(topic, pose);
  editHold = Date.now() + 400;
  viewer.placeStart(colorOf(card.dataset.color), pose);
  queuedPose.set(topic, pose);
  pumpPose(topic);
}

async function pumpPose(topic) {
  if (sendingPose.has(topic)) {
    return;
  }
  sendingPose.add(topic);
  const epoch = poseEpoch.get(topic) || 0;
  try {
    while (queuedPose.has(topic) && (poseEpoch.get(topic) || 0) === epoch) {
      const pose = queuedPose.get(topic);
      queuedPose.delete(topic);
      await post("/api/pose", { topic, ...pose, keep_rotation: false });
    }
  } finally {
    sendingPose.delete(topic);
    if ((poseEpoch.get(topic) || 0) === epoch) {
      editHold = Math.max(editHold, Date.now() + 150);
    }
  }
}

function wheelStep(event, field) {
  const notch = event.deltaMode === 0 ? -event.deltaY / 100 : -event.deltaY;
  let step = isAngle(field) ? 0.1 : 0.01;
  if (event.shiftKey) {
    step *= 0.1;
  } else if (event.ctrlKey || event.metaKey) {
    step *= 10;
  }
  const limit = isAngle(field) ? 5 : 0.5;
  return Math.max(-limit, Math.min(limit, notch * step));
}

function renderCards(state) {
  const key = JSON.stringify(Object.entries(state.lidars).map(([topic, item]) => [
    topic, item.locked, item.confirmed, item.color,
  ])) + String(state.busy) + state.reference;
  if (key !== cardKey || !cards.childElementCount) {
    cardKey = key;
    cards.innerHTML = "";
    for (const [topic, item] of Object.entries(state.lidars)) {
      const card = document.createElement("article");
      card.className = "card";
      card.dataset.topic = topic;
      card.dataset.color = item.color;
      const title = document.createElement("h3");
      const swatch = document.createElement("i");
      swatch.style.background = item.color;
      title.append(swatch, document.createTextNode(item.label + (topic === state.reference ? " · 基准" : "")));
      const lines = document.createElement("p");
      lines.className = "residual";
      const stateLine = document.createElement("p");
      stateLine.className = "state-line";
      const poseBox = document.createElement("div");
      poseBox.className = "pose";
      for (const field of fields) {
        const label = document.createElement("label");
        label.append(document.createTextNode(isAngle(field) ? `${field}°` : field));
        const input = document.createElement("input");
        input.type = "number";
        input.step = isAngle(field) ? "0.1" : "0.01";
        input.dataset.field = field;
        input.title = isAngle(field) ? "单位是度。滚轮调整，Shift 更细，Ctrl 更粗" : "滚轮调整。Shift 更细，Ctrl 更粗";
        input.addEventListener("input", () => schedulePose(topic, card));
        label.addEventListener("wheel", (event) => {
          event.preventDefault();
          const current = Number(input.value);
          if (!Number.isFinite(current)) {
            return;
          }
          const digits = isAngle(field) ? 3 : 3;
          input.value = (current + wheelStep(event, field)).toFixed(digits);
          input.focus();
          pushPose(topic, card);
        }, { passive: false });
        label.append(input);
        poseBox.append(label);
      }
      const actions = document.createElement("div");
      actions.className = "actions";
      const pick = document.createElement("button");
      pick.type = "button";
      pick.className = "pick";
      pick.dataset.topic = topic;
      pick.addEventListener("click", () => {
        pickingTopic = pickingTopic === topic ? "" : topic;
        syncPickButtons();
      });
      const gicp = document.createElement("button");
      gicp.type = "button";
      gicp.textContent = "GICP 精配准";
      gicp.disabled = state.busy || !item.pose;
      gicp.addEventListener("click", () => {
        clearTimeout(poseTimers.get(topic));
        poseEpoch.set(topic, (poseEpoch.get(topic) || 0) + 1);
        queuedPose.delete(topic);
        livePose.delete(topic);
        editHold = 0;
        post("/api/gicp", { topic });
      });
      const confirm = document.createElement("button");
      confirm.type = "button";
      confirm.textContent = "确认起点";
      confirm.disabled = !item.pose || item.confirmed || state.busy;
      confirm.title = item.locked ? "确认当前起点" : "RTK 未达标，确认后可先做雷达相对外参";
      confirm.addEventListener("click", () => post("/api/confirm", { topic }));
      const retry = document.createElement("button");
      retry.type = "button";
      retry.textContent = "换一帧";
      retry.title = "只更换显示的点云，起点数值保持第一帧";
      retry.disabled = state.busy;
      retry.addEventListener("click", () => {
        clearTimeout(poseTimers.get(topic));
        poseEpoch.set(topic, (poseEpoch.get(topic) || 0) + 1);
        queuedPose.delete(topic);
        livePose.delete(topic);
        editHold = 0;
        post("/api/retry", { topic });
      });
      actions.append(pick, gicp, confirm, retry);
      if (topic !== state.reference) {
        const toggle = document.createElement("button");
        toggle.type = "button";
        toggle.className = "cloud-toggle";
        toggle.dataset.topic = topic;
        toggle.addEventListener("click", () => {
          if (hiddenClouds.has(topic)) {
            hiddenClouds.delete(topic);
          } else {
            hiddenClouds.add(topic);
          }
          syncCloudToggle(toggle, topic, item.color);
        });
        syncCloudToggle(toggle, topic, item.color);
        actions.append(toggle);
      }
      card.append(title, lines, stateLine, poseBox, actions);
      cards.append(card);
    }
  }
  for (const [topic, item] of Object.entries(state.lidars)) {
    const card = cards.querySelector(`article[data-topic="${cssEscape(topic)}"]`);
    if (!card) {
      continue;
    }
    const lines = card.querySelector(".residual");
    lines.textContent = item.median === undefined
      ? (item.detail || item.status || "")
      : `${item.method || ""}  近处 ${item.median} m，0.3 m 内 ${item.inlier_0_3}，1 m 内 ${item.inlier_1}。${item.detail || ""}`;
    card.querySelector(".state-line").textContent = item.confirmed ? "起点已确认" : "起点未确认";
    fillPose(card, item.pose, topic);
    const gicp = [...card.querySelectorAll("button")].find((button) => button.textContent === "GICP 精配准");
    if (gicp) {
      gicp.disabled = state.busy || !item.pose;
    }
    const confirm = [...card.querySelectorAll("button")].find((button) => button.textContent === "确认起点");
    if (confirm) {
      confirm.disabled = !item.pose || item.confirmed || state.busy;
      confirm.title = item.locked ? "确认当前起点" : "RTK 未达标，确认后可先做雷达相对外参";
    }
  }
  if (pickingTopic && !state.lidars[pickingTopic]) {
    pickingTopic = "";
  }
  syncPickButtons();
}

function syncCloudToggle(button, topic, color) {
  const hidden = hiddenClouds.has(topic);
  button.textContent = hidden ? "不显示" : "显示";
  button.classList.toggle("off", hidden);
  button.title = hidden ? "当前不显示该雷达点云，点击后显示" : "当前显示该雷达点云，点击后不显示";
  viewer.setVisible(colorOf(color), !hidden);
}

function applyCloudVisibility(lidars) {
  for (const [topic, item] of Object.entries(lidars || {})) {
    if (hiddenClouds.has(topic)) {
      viewer.setVisible(colorOf(item.color), false);
    }
  }
}

function syncPickButtons() {
  canvas.classList.toggle("picking", Boolean(pickingTopic));
  for (const button of cards.querySelectorAll("button.pick")) {
    const active = button.dataset.topic === pickingTopic;
    button.classList.toggle("picking", active);
    button.textContent = active ? "取消点选" : "点选位置";
  }
  const active = pickingTopic ? cards.querySelector(`article[data-topic="${cssEscape(pickingTopic)}"] h3`) : null;
  hint.textContent = active
    ? `正在为${active.textContent}点选位置。单击地图发布该点，拖动仍用于旋转。`
    : "白点是下采样地图，橙线是 RTK。红轴 X、绿轴 Y、蓝轴 Z。拖动旋转，滚轮缩放，Shift 或右键平移。数值上滚轮可微调，点云实时跟着动。";
}

function renderStarts(state) {
  const visible = state.phase === "select" || state.phase === "review" || state.phase === "done";
  starts.hidden = !visible;
  const saved = state.saved_starts || {};
  startsNote.textContent = saved.available
    ? `这组数据已保存 ${saved.count} 颗雷达的起点${saved.saved_at ? `，${saved.saved_at}` : ""}。`
    : "当前这组地图和数据包还没有保存起点。";
  const hasPose = Object.values(state.lidars || {}).some((item) => item.pose);
  saveStartsButton.disabled = state.busy || !hasPose;
  restoreStartsButton.disabled = state.busy || !saved.available;
}

function meterText(value) {
  return Number(value).toFixed(3);
}

async function copyCalibFile(text, sourcePath) {
  const base = String(sourcePath || "map_calib.yaml").split("/").pop().replace(/\.yaml$/i, "");
  const filename = `${base}_base_link.yaml`;
  try {
    const response = await fetch("http://127.0.0.1:8767/copy", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ filename, text }),
    });
    if (!response.ok) {
      throw new Error(await response.text());
    }
    notice.textContent = "已复制标定文件，可在文件夹中粘贴";
  } catch (error) {
    await copyText(text);
    notice.textContent = "文件夹无法粘贴这份文件，已改为复制文本";
  }
}

async function copyText(text) {
  try {
    await navigator.clipboard.writeText(text);
  } catch (error) {
    const area = document.createElement("textarea");
    area.value = text;
    document.body.append(area);
    area.select();
    document.execCommand("copy");
    area.remove();
  }
  notice.textContent = "已复制";
}

function copyCell(text) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "copy";
  button.textContent = text;
  button.title = "点击复制";
  button.addEventListener("click", () => copyText(text));
  return button;
}

function resultRows(value) {
  const rows = [];
  const push = (name, pose) => {
    if (!pose) {
      return;
    }
    rows.push({
      name,
      x: meterText(pose.x),
      y: meterText(pose.y),
      z: meterText(pose.z),
      roll: degText(pose.roll),
      pitch: degText(pose.pitch),
      yaw: degText(pose.yaw),
      count: pose.count === undefined ? "" : String(pose.count),
      spread: pose.spread === undefined ? "" : meterText(pose.spread),
    });
  };
  for (const [name, pose] of Object.entries(value.lidars || {})) {
    push(`${name} 相对基准`, pose);
  }
  if (value.rtk) {
    push(`RTK 在 ${value.reference} 下`, value.rtk);
  }
  if (value.lidar_in_rtk) {
    push(`${value.reference} 在 RTK 下`, value.lidar_in_rtk);
  }
  for (const [name, pose] of Object.entries(value.baseline || {})) {
    push(`${name} 在 baseline`, pose);
  }
  return rows;
}

function degText(radians) {
  return toDegrees(radians).toFixed(3);
}

function yamlNumber(value) {
  return Number(value).toFixed(10);
}

function standardYaml(value) {
  const entries = Object.entries(value.baseline || {}).filter(([, pose]) => pose && pose.x !== undefined);
  const lines = [
    "# 各雷达外参：xxx_base_link 相对 base_link",
    "# 单位：m / rad（RPY 为固定轴 XYZ）",
    "# X+ 前，Y+ 左，Z+ 上。",
    "# 基准雷达高度按 baseline 写成 1.0，其余雷达相对基准不变。",
    "",
  ];
  let section = "";
  for (const [frame, pose] of entries) {
    const next = String(frame).startsWith("suteng_") ? "blind" : "long";
    if (next !== section) {
      section = next;
      lines.push(next === "blind" ? "# ---- 补盲雷达----" : "# ---- 远距雷达----");
    }
    const name = String(frame).endsWith("_base_link") ? frame : `${frame}_base_link`;
    lines.push(`${name}:`);
    for (const field of fields) {
      lines.push(`  ${field}: ${yamlNumber(pose[field])}`);
    }
  }
  lines.push("");
  return lines.join("\n");
}

function renderResult(value) {
  showResultButton.hidden = !value;
  if (!value) {
    resultDialog.hidden = true;
    resultBody.innerHTML = "";
    resultText = "";
    resultKey = "";
    resultPath = "";
    return;
  }
  const rows = resultRows(value);
  const yaml = standardYaml(value);
  resultText = yaml;
  resultPath = value.path || "";
  if (resultKey === yaml) {
    return;
  }
  resultKey = yaml;
  resultBody.innerHTML = "";
    const note = document.createElement("p");
    note.className = "panel-note";
    note.textContent = `基准 ${value.reference}。文件 ${value.path}。上表角度为度。下面是标准格式的原始 yaml，单位是弧度。复制标定文件会把这段 yaml 放入剪贴板。`;
    const table = document.createElement("table");
    const head = document.createElement("tr");
    for (const title of ["关系", "x", "y", "z", "roll°", "pitch°", "yaw°", "帧数", "离散 m"]) {
      const cell = document.createElement("th");
      cell.textContent = title;
      head.append(cell);
    }
    table.append(head);
    for (const row of rows) {
      const line = document.createElement("tr");
      const values = [row.name, row.x, row.y, row.z, row.roll, row.pitch, row.yaw, row.count, row.spread];
      for (const text of values) {
        const cell = document.createElement("td");
        if (text) {
          cell.append(copyCell(text));
        }
        line.append(cell);
      }
      table.append(line);
    }
    const fileTitle = document.createElement("h3");
    fileTitle.textContent = "标定文件";
    const file = document.createElement("pre");
    file.className = "yaml-file";
    file.textContent = yaml;
    resultBody.append(note, table, fileTitle, file);
    resultDialog.hidden = false;
}

async function poll() {
  const response = await fetch("/api/state");
  const state = await response.json();
  const ready = state.phase !== "loading" && state.phase !== "error";
  summary.textContent = state.map_points
    ? `${state.frame}。地图 ${state.map_points} 点，RTK ${state.rtk_count} 个，航向变化 ${state.yaw_span}°。${state.note}`
    : state.message;
  if (state.view_revision !== shownView && !viewBusy && Date.now() >= editHold && sendingPose.size === 0) {
    viewBusy = true;
    const revision = state.view_revision;
    const anchors = Object.entries(state.lidars)
      .filter(([, item]) => item.pose)
      .map(([topic, item]) => ({
        topic,
        color: colorOf(item.color),
        pose: item.view_pose || item.pose,
        start: item.pose,
      }));
    fetch("/api/view")
      .then((response) => response.arrayBuffer())
      .then((buffer) => {
        if (Date.now() < editHold || sendingPose.size > 0) {
          return;
        }
        viewer.setLayers(PointView.parse(buffer));
        applyCloudVisibility(state.lidars);
        for (const anchor of anchors) {
          viewer.anchorLayer(anchor.color, anchor.pose, anchor.start);
          const live = livePose.get(anchor.topic);
          if (live && anchor.start && !poseClose(live, anchor.start)) {
            viewer.placeStart(anchor.color, live);
          }
        }
        shownView = revision;
      })
      .catch(() => {})
      .finally(() => {
        viewBusy = false;
      });
  }
  if (state.error) {
    notice.textContent = state.error;
  } else if (notice.textContent !== "已复制") {
    notice.textContent = "";
  }
  renderSources(state);
  renderStarts(state);
  picker.hidden = !(ready && state.phase === "select");
  if (ready && state.phase === "select") {
    renderTopics(state);
  }
  if (state.phase === "review" || state.phase === "optimizing" || state.phase === "done" || state.phase === "aligning") {
    renderCards(state);
  } else if (cards.childElementCount) {
    cardKey = "";
    cards.innerHTML = "";
    pickingTopic = "";
    syncPickButtons();
  }
  const chosen = Object.values(state.lidars);
  const allConfirmed = chosen.length > 0 && chosen.every((item) => item.confirmed);
  alignButton.disabled = state.phase !== "select" || state.busy;
  const referenceItem = state.lidars[state.reference];
  const relativeOnly = Boolean(referenceItem && !referenceItem.locked);
  optimizeButton.textContent = relativeOnly ? "相对外参标定" : "开始逐帧优化";
  optimizeButton.title = relativeOnly
    ? "基准未达到 RTK 条件，用各雷达当前起点标定相对外参"
    : "用已确认的起点做逐帧地图匹配";
  optimizeButton.disabled = !allConfirmed || state.busy;
  resetButton.disabled = state.phase === "sources" || state.phase === "loading" || !state.map_points || state.busy;
  changeButton.disabled = state.busy || state.phase === "sources" || state.phase === "loading";
  log.textContent = state.log.join("\n");
  renderResult(state.result);
}

function storedSize(name, fallback) {
  const saved = localStorage.getItem(name);
  const value = saved || fallback;
  document.documentElement.style.setProperty(name, value);
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

storedSize("--side-width", "560px");
storedSize("--output-height", "240px");
storedSize("--result-width", "820px");
storedSize("--result-height", "520px");
dragSize(document.querySelector("#side-resizer"), () => document.querySelector("#side").getBoundingClientRect().width, (origin, dx) => {
  const width = Math.max(420, Math.min(980, origin + dx));
  document.documentElement.style.setProperty("--side-width", `${width}px`);
  localStorage.setItem("--side-width", `${width}px`);
});
dragSize(document.querySelector("#output .resize-y"), () => output.getBoundingClientRect().height, (origin, _dx, dy) => {
  const height = Math.max(140, Math.min(window.innerHeight * 0.7, origin - dy));
  document.documentElement.style.setProperty("--output-height", `${height}px`);
  localStorage.setItem("--output-height", `${height}px`);
});
dragSize(document.querySelector("#result-window .resize-corner"), () => ({
  width: resultWindow.getBoundingClientRect().width,
  height: resultWindow.getBoundingClientRect().height,
}), (origin, dx, dy) => {
  const width = Math.max(520, Math.min(window.innerWidth - 48, origin.width + dx));
  const height = Math.max(280, Math.min(window.innerHeight - 48, origin.height + dy));
  document.documentElement.style.setProperty("--result-width", `${width}px`);
  document.documentElement.style.setProperty("--result-height", `${height}px`);
  localStorage.setItem("--result-width", `${width}px`);
  localStorage.setItem("--result-height", `${height}px`);
});
showResultButton.addEventListener("click", () => {
  resultDialog.hidden = false;
});
closeResultButton.addEventListener("click", () => {
  resultDialog.hidden = true;
});
copyResultButton.addEventListener("click", () => copyCalibFile(resultText, resultPath));
resultDialog.addEventListener("click", (event) => {
  if (event.target === resultDialog) {
    resultDialog.hidden = true;
  }
});

setInterval(() => poll().catch(() => {}), 1000);
poll().catch(() => {});
