const rad = Math.PI / 180;
const sections = [...document.querySelectorAll(".lidar")];

function ensureStepControls() {
  if (document.querySelector("#step-trans")) {
    return;
  }
  const controls = document.querySelector(".controls");
  const trans = document.createElement("label");
  trans.className = "step";
  trans.innerHTML = '平移步长 m<input id="step-trans" type="number" min="0.0001" step="0.01" value="0.01">';
  const angle = document.createElement("label");
  angle.className = "step";
  angle.innerHTML = '角度步长 °<input id="step-angle" type="number" min="0.001" step="0.1" value="0.1">';
  controls.prepend(angle);
  controls.prepend(trans);
}

ensureStepControls();
const stepTrans = document.querySelector("#step-trans");
const stepAngle = document.querySelector("#step-angle");
for (const input of [stepTrans, stepAngle]) {
  const saved = localStorage.getItem(input.id);
  if (saved && Number(saved) > 0) {
    input.value = saved;
  }
  input.addEventListener("change", () => localStorage.setItem(input.id, input.value));
}

function stepFor(input) {
  const control = input.dataset.angle ? stepAngle : stepTrans;
  const value = Number(control.value);
  if (Number.isFinite(value) && value > 0) {
    return value;
  }
  return input.dataset.angle ? 0.1 : 0.01;
}
const pauseButton = document.querySelector("#pause");
const playState = document.querySelector("#play-state");
const notice = document.querySelector("#notice");
let applying = false;

function poseFrom(section) {
  const pose = { id: section.dataset.id };
  for (const input of section.querySelectorAll("input")) {
    const value = Number(input.value);
    if (!Number.isFinite(value)) {
      return null;
    }
    pose[input.dataset.field] = input.dataset.angle ? value * rad : value;
  }
  return pose;
}

function fill(poses) {
  applying = true;
  for (const section of sections) {
    const pose = poses[section.dataset.id];
    if (!pose) {
      continue;
    }
    for (const input of section.querySelectorAll("input")) {
      const value = pose[input.dataset.field];
      input.value = (input.dataset.angle ? value / rad : value).toFixed(input.dataset.angle ? 3 : 4);
    }
  }
  applying = false;
}

async function push(section) {
  if (applying) {
    return;
  }
  const pose = poseFrom(section);
  if (!pose) {
    notice.textContent = "有无效数字";
    return;
  }
  const response = await fetch("/api/extrinsics", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(pose),
  });
  notice.textContent = response.ok ? "" : "外参没有写入";
}

function setPaused(paused) {
  if (paused === null || paused === undefined) {
    playState.textContent = "等待播放器";
    pauseButton.textContent = "暂停";
    pauseButton.classList.remove("paused");
    return;
  }
  playState.textContent = paused ? "已暂停" : "播放中";
  pauseButton.textContent = paused ? "继续" : "暂停";
  pauseButton.classList.toggle("paused", paused);
}

async function togglePause() {
  const response = await fetch("/api/toggle_pause", { method: "POST" });
  if (!response.ok) {
    notice.textContent = "播放器还没就绪";
    return;
  }
  const body = await response.json();
  setPaused(body.paused);
}

for (const section of sections) {
  for (const input of section.querySelectorAll("input")) {
    input.addEventListener("input", () => push(section));
    input.addEventListener("wheel", (event) => {
      event.preventDefault();
      const current = Number(input.value);
      if (!Number.isFinite(current)) {
        return;
      }
      const direction = event.deltaY < 0 ? 1 : -1;
      const digits = input.dataset.angle ? 4 : 5;
      input.value = (current + direction * stepFor(input)).toFixed(digits);
      input.focus();
      push(section);
    }, { passive: false });
  }
}

pauseButton.addEventListener("click", togglePause);
document.querySelector("#reset").addEventListener("click", async () => {
  const response = await fetch("/api/reset", { method: "POST" });
  const body = await response.json();
  fill(body.poses);
  notice.textContent = "已恢复初始外参";
});
document.querySelector("#save").addEventListener("click", async () => {
  const response = await fetch("/api/save", { method: "POST" });
  const body = await response.json();
  notice.textContent = response.ok ? `已保存 ${body.path}` : "保存失败";
});

window.addEventListener("keydown", (event) => {
  if (event.code !== "Space" || event.repeat) {
    return;
  }
  if (event.target.tagName === "TEXTAREA") {
    return;
  }
  event.preventDefault();
  togglePause();
});

async function refresh() {
  const response = await fetch("/api/state");
  if (!response.ok) {
    return;
  }
  const body = await response.json();
  if (document.activeElement.tagName !== "INPUT") {
    fill(body.poses);
  }
  for (const section of sections) {
    const stat = body.stats[section.dataset.id];
    const count = section.querySelector(".count");
    count.textContent = stat && stat.count ? `${stat.points} 点 · 第 ${stat.count} 帧` : "尚无点云";
  }
  if (document.activeElement !== pauseButton) {
    setPaused(body.paused);
  }
}

refresh();
setInterval(refresh, 500);
