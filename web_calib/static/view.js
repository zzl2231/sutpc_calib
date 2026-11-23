const PointView = (() => {
  const vertex = `
    attribute vec3 position;
    uniform mat4 mvp;
    uniform float pointSize;
    void main() {
      gl_Position = mvp * vec4(position, 1.0);
      gl_PointSize = pointSize;
    }`;
  const fragment = `
    precision mediump float;
    uniform vec3 color;
    void main() {
      gl_FragColor = vec4(color, 1.0);
    }`;

  function shader(gl, type, source) {
    const item = gl.createShader(type);
    gl.shaderSource(item, source);
    gl.compileShader(item);
    return item;
  }

  function multiply(a, b) {
    const out = new Float32Array(16);
    for (let column = 0; column < 4; column += 1) {
      for (let row = 0; row < 4; row += 1) {
        out[column * 4 + row] =
          a[row] * b[column * 4] +
          a[4 + row] * b[column * 4 + 1] +
          a[8 + row] * b[column * 4 + 2] +
          a[12 + row] * b[column * 4 + 3];
      }
    }
    return out;
  }

  function perspective(aspect) {
    const fovy = Math.PI / 4;
    const near = 0.1;
    const far = 5000;
    const f = 1 / Math.tan(fovy / 2);
    const out = new Float32Array(16);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = (far + near) / (near - far);
    out[11] = -1;
    out[14] = (2 * far * near) / (near - far);
    return out;
  }

  function lookAt(eye, target) {
    const forward = normalize([
      eye[0] - target[0],
      eye[1] - target[1],
      eye[2] - target[2],
    ]);
    const up = [0, 0, 1];
    let right = cross(up, forward);
    if (length(right) < 1e-4) {
      right = [1, 0, 0];
    }
    right = normalize(right);
    const cameraUp = normalize(cross(forward, right));
    const out = new Float32Array(16);
    out[0] = right[0];
    out[1] = cameraUp[0];
    out[2] = forward[0];
    out[4] = right[1];
    out[5] = cameraUp[1];
    out[6] = forward[1];
    out[8] = right[2];
    out[9] = cameraUp[2];
    out[10] = forward[2];
    out[12] = -dot(right, eye);
    out[13] = -dot(cameraUp, eye);
    out[14] = -dot(forward, eye);
    out[15] = 1;
    return out;
  }

  function length(v) {
    return Math.hypot(v[0], v[1], v[2]);
  }

  function normalize(v) {
    const scale = length(v) || 1;
    return [v[0] / scale, v[1] / scale, v[2] / scale];
  }

  function cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
  }

  function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  function attach(canvas) {
    const gl = canvas.getContext("webgl", { antialias: false, alpha: false });
    const program = gl.createProgram();
    gl.attachShader(program, shader(gl, gl.VERTEX_SHADER, vertex));
    gl.attachShader(program, shader(gl, gl.FRAGMENT_SHADER, fragment));
    gl.linkProgram(program);
    const positionLoc = gl.getAttribLocation(program, "position");
    const mvpLoc = gl.getUniformLocation(program, "mvp");
    const sizeLoc = gl.getUniformLocation(program, "pointSize");
    const colorLoc = gl.getUniformLocation(program, "color");
    const view = {
      yaw: 0.8,
      pitch: 0.7,
      distance: 80,
      target: [0, 0, 0],
      fitted: false,
      layers: [],
    };

    function resize() {
      const ratio = window.devicePixelRatio || 1;
      const width = Math.max(1, canvas.clientWidth);
      const height = Math.max(1, canvas.clientHeight);
      canvas.width = Math.floor(width * ratio);
      canvas.height = Math.floor(height * ratio);
      gl.viewport(0, 0, canvas.width, canvas.height);
      draw();
    }

    const axisBuffer = gl.createBuffer();
    const fovy = Math.PI / 4;

    function eye() {
      const horizontal = Math.cos(view.pitch);
      return [
        view.target[0] + view.distance * horizontal * Math.sin(view.yaw),
        view.target[1] + view.distance * horizontal * Math.cos(view.yaw),
        view.target[2] + view.distance * Math.sin(view.pitch),
      ];
    }

    function basis(yaw, pitch) {
      const horizontal = Math.cos(pitch);
      const forward = normalize([
        horizontal * Math.sin(yaw),
        horizontal * Math.cos(yaw),
        Math.sin(pitch),
      ]);
      let right = [-forward[1], forward[0], 0];
      const span = Math.hypot(right[0], right[1]);
      right = span < 1e-4 ? [1, 0, 0] : [right[0] / span, right[1] / span, 0];
      return { right, up: normalize(cross(forward, right)) };
    }

    function pixelsToWorld(distance) {
      return (2 * distance * Math.tan(fovy / 2)) / Math.max(1, canvas.clientHeight);
    }

    function draw() {
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.17, 0.17, 0.17, 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);
      gl.useProgram(program);
      const mvp = multiply(perspective(canvas.width / canvas.height), lookAt(eye(), view.target));
      view.mvp = mvp;
      gl.uniformMatrix4fv(mvpLoc, false, mvp);
      for (const layer of view.layers) {
        if (layer.hidden) {
          continue;
        }
        gl.bindBuffer(gl.ARRAY_BUFFER, layer.buffer);
        gl.enableVertexAttribArray(positionLoc);
        gl.vertexAttribPointer(positionLoc, 3, gl.FLOAT, false, 0, 0);
        gl.uniform3f(colorLoc, layer.color[0], layer.color[1], layer.color[2]);
        if (layer.line && layer.count > 1) {
          gl.uniform1f(sizeLoc, 1);
          gl.drawArrays(gl.LINE_STRIP, 0, layer.count);
        }
        gl.uniform1f(sizeLoc, layer.size);
        gl.drawArrays(gl.POINTS, 0, layer.count);
      }
      drawAxes();
    }

    function drawAxes() {
      const length = Math.max(view.distance * 0.12, 3);
      const origin = view.target;
      const verts = new Float32Array([
        origin[0], origin[1], origin[2], origin[0] + length, origin[1], origin[2],
        origin[0], origin[1], origin[2], origin[0], origin[1] + length, origin[2],
        origin[0], origin[1], origin[2], origin[0], origin[1], origin[2] + length,
      ]);
      gl.bindBuffer(gl.ARRAY_BUFFER, axisBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, verts, gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(positionLoc);
      gl.vertexAttribPointer(positionLoc, 3, gl.FLOAT, false, 0, 0);
      gl.uniform1f(sizeLoc, 1);
      gl.disable(gl.DEPTH_TEST);
      const colors = [[1, 0.25, 0.2], [0.25, 0.9, 0.35], [0.3, 0.55, 1]];
      for (let index = 0; index < colors.length; index += 1) {
        gl.uniform3f(colorLoc, colors[index][0], colors[index][1], colors[index][2]);
        gl.drawArrays(gl.LINES, index * 2, 2);
      }
      gl.enable(gl.DEPTH_TEST);
    }

    function fit(points) {
      let min = [Infinity, Infinity, Infinity];
      let max = [-Infinity, -Infinity, -Infinity];
      const stride = Math.max(1, Math.floor(points.length / 3 / 4000));
      for (let index = 0; index < points.length; index += 3 * stride) {
        for (let axis = 0; axis < 3; axis += 1) {
          min[axis] = Math.min(min[axis], points[index + axis]);
          max[axis] = Math.max(max[axis], points[index + axis]);
        }
      }
      view.target = [
        (min[0] + max[0]) / 2,
        (min[1] + max[1]) / 2,
        (min[2] + max[2]) / 2,
      ];
      view.distance = Math.max(Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) * 0.7, 5);
      view.fitted = true;
    }

    let drag = null;
    canvas.addEventListener("pointerdown", (event) => {
      canvas.setPointerCapture(event.pointerId);
      drag = {
        x: event.clientX,
        y: event.clientY,
        button: event.button,
        yaw: view.yaw,
        pitch: view.pitch,
        distance: view.distance,
        target: view.target.slice(),
      };
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!drag) {
        return;
      }
      const dx = event.clientX - drag.x;
      const dy = event.clientY - drag.y;
      if (Math.hypot(dx, dy) < 3) {
        return;
      }
      if (drag.button === 2 || event.shiftKey) {
        const { right, up } = basis(drag.yaw, drag.pitch);
        const scale = pixelsToWorld(drag.distance);
        view.target = [
          drag.target[0] - (dx * right[0] - dy * up[0]) * scale,
          drag.target[1] - (dx * right[1] - dy * up[1]) * scale,
          drag.target[2] - (dx * right[2] - dy * up[2]) * scale,
        ];
      } else {
        const step = fovy / Math.max(1, canvas.clientHeight);
        view.yaw = drag.yaw + dx * step / Math.max(0.2, Math.cos(drag.pitch));
        view.pitch = Math.max(-1.2, Math.min(1.2, drag.pitch + dy * step));
      }
      draw();
    });
    let pickHandler = null;
    canvas.addEventListener("pointerup", (event) => {
      if (!drag) {
        return;
      }
      const moved = Math.hypot(event.clientX - drag.x, event.clientY - drag.y);
      const button = drag.button;
      drag = null;
      if (button !== 0 || event.shiftKey || moved >= 5 || !pickHandler) {
        return;
      }
      const point = pickAt(event.clientX, event.clientY);
      if (point) {
        pickHandler(point);
      }
    });
    canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      view.distance *= event.deltaY > 0 ? 1.1 : 0.9;
      view.distance = Math.max(1, Math.min(4000, view.distance));
      draw();
    }, { passive: false });
    window.addEventListener("resize", resize);
    if (typeof ResizeObserver !== "undefined") {
      const observer = new ResizeObserver(() => resize());
      observer.observe(canvas);
    }

    function pickAt(clientX, clientY) {
      const layer = view.layers[0];
      if (!layer || !layer.points || !view.mvp) {
        return null;
      }
      const rect = canvas.getBoundingClientRect();
      const sx = clientX - rect.left;
      const sy = clientY - rect.top;
      const mvp = view.mvp;
      const points = layer.points;
      const stride = Math.max(1, Math.floor(points.length / 3 / 25000));
      let best = null;
      let bestDepth = Infinity;
      const limit = 16 * 16;
      for (let index = 0; index < points.length; index += 3 * stride) {
        const x = points[index];
        const y = points[index + 1];
        const z = points[index + 2];
        const clipX = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
        const clipY = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
        const clipZ = mvp[2] * x + mvp[6] * y + mvp[10] * z + mvp[14];
        const clipW = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
        if (clipW <= 0.01) {
          continue;
        }
        const px = (clipX / clipW * 0.5 + 0.5) * rect.width;
        const py = (-clipY / clipW * 0.5 + 0.5) * rect.height;
        const distance = (px - sx) * (px - sx) + (py - sy) * (py - sy);
        const depth = clipZ / clipW;
        if (distance <= limit && depth < bestDepth) {
          bestDepth = depth;
          best = [x, y, z];
        }
      }
      return best;
    }

    function rotation(roll, pitch, yaw) {
      const cr = Math.cos(roll);
      const sr = Math.sin(roll);
      const cp = Math.cos(pitch);
      const sp = Math.sin(pitch);
      const cy = Math.cos(yaw);
      const sy = Math.sin(yaw);
      const a00 = cp;
      const a01 = sp * sr;
      const a02 = sp * cr;
      const a11 = cr;
      const a12 = -sr;
      const a20 = -sp;
      const a21 = cp * sr;
      const a22 = cp * cr;
      return [
        cy * a00, cy * a01 - sy * a11, cy * a02 - sy * a12,
        sy * a00, sy * a01 + cy * a11, sy * a02 + cy * a12,
        a20, a21, a22,
      ];
    }

    function sameColor(left, right) {
      return Math.abs(left[0] - right[0]) < 0.02
        && Math.abs(left[1] - right[1]) < 0.02
        && Math.abs(left[2] - right[2]) < 0.02;
    }

    function layerByColor(color) {
      return view.layers.find((layer) => !layer.line && layer.basePose && sameColor(layer.color, color));
    }

    function matrixOf(pose) {
      const r = rotation(pose.roll, pose.pitch, pose.yaw);
      const m = new Float32Array(16);
      m[0] = r[0];
      m[1] = r[3];
      m[2] = r[6];
      m[4] = r[1];
      m[5] = r[4];
      m[6] = r[7];
      m[8] = r[2];
      m[9] = r[5];
      m[10] = r[8];
      m[12] = pose.x;
      m[13] = pose.y;
      m[14] = pose.z;
      m[15] = 1;
      return m;
    }

    function poseOf(m) {
      const pitch = Math.atan2(-m[2], Math.hypot(m[6], m[10]));
      return {
        x: m[12],
        y: m[13],
        z: m[14],
        roll: Math.atan2(m[6], m[10]),
        pitch,
        yaw: Math.atan2(m[1], m[0]),
      };
    }

    function invertRigid(m) {
      const out = new Float32Array(16);
      out[0] = m[0];
      out[1] = m[4];
      out[2] = m[8];
      out[4] = m[1];
      out[5] = m[5];
      out[6] = m[9];
      out[8] = m[2];
      out[9] = m[6];
      out[10] = m[10];
      out[12] = -(out[0] * m[12] + out[4] * m[13] + out[8] * m[14]);
      out[13] = -(out[1] * m[12] + out[5] * m[13] + out[9] * m[14]);
      out[14] = -(out[2] * m[12] + out[6] * m[13] + out[10] * m[14]);
      out[15] = 1;
      return out;
    }

    function copyPose(pose) {
      return {
        x: pose.x, y: pose.y, z: pose.z, roll: pose.roll, pitch: pose.pitch, yaw: pose.yaw,
      };
    }

    function anchorLayer(color, pose, start) {
      const layer = view.layers.find((item) => !item.line && sameColor(item.color, color));
      if (!layer) {
        return;
      }
      layer.basePoints = new Float32Array(layer.points);
      layer.basePose = copyPose(pose);
      layer.viewPose = copyPose(pose);
      layer.startPose = copyPose(start || pose);
    }

    function placeStart(color, start) {
      const layer = layerByColor(color);
      if (!layer || !layer.viewPose || !layer.startPose) {
        moveLayer(color, start);
        return;
      }
      const display = multiply(
        multiply(matrixOf(layer.viewPose), invertRigid(matrixOf(layer.startPose))),
        matrixOf(start),
      );
      moveLayer(color, poseOf(display));
    }

    function moveLayer(color, pose) {
      const layer = layerByColor(color);
      if (!layer) {
        return;
      }
      const from = rotation(layer.basePose.roll, layer.basePose.pitch, layer.basePose.yaw);
      const to = rotation(pose.roll, pose.pitch, pose.yaw);
      const source = layer.basePoints;
      const output = layer.moved || new Float32Array(source.length);
      layer.moved = output;
      for (let index = 0; index < source.length; index += 3) {
        const dx = source[index] - layer.basePose.x;
        const dy = source[index + 1] - layer.basePose.y;
        const dz = source[index + 2] - layer.basePose.z;
        const lx = from[0] * dx + from[3] * dy + from[6] * dz;
        const ly = from[1] * dx + from[4] * dy + from[7] * dz;
        const lz = from[2] * dx + from[5] * dy + from[8] * dz;
        output[index] = to[0] * lx + to[1] * ly + to[2] * lz + pose.x;
        output[index + 1] = to[3] * lx + to[4] * ly + to[5] * lz + pose.y;
        output[index + 2] = to[6] * lx + to[7] * ly + to[8] * lz + pose.z;
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, layer.buffer);
      gl.bufferData(gl.ARRAY_BUFFER, output, gl.DYNAMIC_DRAW);
      draw();
    }

    return {
      setPickHandler(handler) {
        pickHandler = handler;
      },
      anchorLayer,
      moveLayer,
      placeStart,
      setVisible(color, visible) {
        for (const layer of view.layers) {
          if (!layer.line && sameColor(layer.color, color)) {
            layer.hidden = !visible;
          }
        }
        draw();
      },
      setLayers(layers) {
        for (const previous of view.layers) {
          gl.deleteBuffer(previous.buffer);
        }
        view.layers = layers.map((layer) => {
          const buffer = gl.createBuffer();
          gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
          gl.bufferData(gl.ARRAY_BUFFER, layer.points, gl.STATIC_DRAW);
          return { buffer, count: layer.points.length / 3, color: layer.color, size: layer.size, line: layer.line, points: layer.points };
        });
        if (!view.fitted && layers.length) {
          fit(layers[0].points);
        }
        resize();
      },
    };
  }

  function parse(buffer) {
    const bytes = new Uint8Array(buffer);
    const magic = String.fromCharCode(bytes[0], bytes[1], bytes[2], bytes[3]);
    if (magic !== "WPV1") {
      return [];
    }
    const header = new DataView(buffer);
    const count = header.getUint32(8, true);
    const layers = [];
    let offset = 12;
    for (let index = 0; index < count; index += 1) {
      const points = header.getUint32(offset, true);
      const color = [
        bytes[offset + 4] / 255,
        bytes[offset + 5] / 255,
        bytes[offset + 6] / 255,
      ];
      const size = bytes[offset + 7];
      const line = bytes[offset + 8] === 1;
      offset += 12;
      const xyz = new Float32Array(buffer, offset, points * 3);
      layers.push({ points: new Float32Array(xyz), color, size, line });
      offset += points * 12;
    }
    return layers;
  }

  return { attach, parse };
})();
