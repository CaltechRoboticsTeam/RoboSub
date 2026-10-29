/* Standard Gamepad input; commands and watchdogs are implemented by ROS. */
(async () => {
  const panel = document.querySelector("#gamepad");
  const status = document.querySelector("#gamepad-status");
  const preview = document.querySelector("#gamepad-preview");
  const select = document.querySelector("#gamepad-device");
  const enable = document.querySelector("#gamepad-enable");
  const stop = document.querySelector("#gamepad-stop");
  let settings;
  try {
    settings = await request("/api/gamepad");
  } catch (error) {
    return;
  }
  if (settings.source === "off") return;
  panel.hidden = false;
  const mode = settings.simulation ? "Simulation" :
    settings.actuator_transport === "teensy_usb" ?
      "Real mode: Teensy motor output" : "Real mode: numeric previews";
  document.querySelector("#gamepad-mode").textContent =
    `${mode}; ${settings.vehicle}; ${settings.max_effort * 100}% command limit.`;

  async function showPreview() {
    try {
      const data = await request("/api/gamepad");
      const numbers = values => values.map(value => value.toFixed(3)).join("  ");
      const command = data.command
        ? numbers(data.command) : "waiting / stale";
      const motors = data.preview ? numbers(data.preview) : "waiting / stale";
      preview.textContent = [
        `RB enabled: ${data.enabled === true ? "yes" : "no"}`,
        `Body effort [forward, left, up, turn left]: ${command}`,
        `${data.preview_names.map((name, i) =>
          `${data.preview_ids[i]}:${name}`).join("  ")} ` +
          `(${data.preview_units}; ` +
          `${data.actuator_transport === "teensy_usb" ? "USB ACK" : "preview"})`,
        motors,
      ].join("\n");
    } catch (error) {
      preview.textContent = `Robot connection: ${error.message}`;
    }
  }
  // Controller telemetry refreshes separately from manual tuning requests.
  setInterval(showPreview, 200);
  showPreview();
  if (settings.source === "local") {
    document.querySelector("#gamepad-browser").hidden = true;
    status.textContent = "Input comes from the controller on the Linux computer.";
    return;
  }
  if (!navigator.getGamepads || !window.isSecureContext) {
    status.textContent = "Open this page on http://127.0.0.1 through SSH " +
      "using a browser with Gamepad support.";
    enable.disabled = true;
    return;
  }
  const session = crypto.randomUUID();
  let sequence = 0;
  let armed = false;
  let centered = false;
  let busy = false;
  let pending;
  let selectedId;

  function devices() {
    const previous = select.value;
    select.replaceChildren();
    for (const pad of navigator.getGamepads()) {
      if (!pad) continue;
      const option = document.createElement("option");
      option.value = String(pad.index);
      option.textContent = pad.id;
      select.append(option);
    }
    if ([...select.options].some(option => option.value === previous)) {
      select.value = previous;
    }
  }

  function sendFrame(axes, held, options = {}) {
    return request("/api/gamepad", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Tuning-Token": token,
      },
      body: JSON.stringify({session, sequence: sequence++, axes, held}),
      ...options,
    });
  }

  function disable(message) {
    armed = false;
    centered = false;
    enable.disabled = false;
    select.disabled = false;
    if (pending) pending.abort();
    status.textContent = message;
    // A higher sequence stop supersedes an earlier request still in flight.
    sendFrame([0, 0, 0, 0], false, {keepalive: true}).catch(() => {});
  }

  enable.onclick = () => {
    devices();
    const pad = navigator.getGamepads()[Number(select.value)];
    if (!pad || pad.mapping !== "standard" || pad.axes.length < 4 ||
        pad.buttons.length < 6) {
      status.textContent = "Connect an Xbox controller and press a button. " +
        "The browser must report the standard gamepad layout.";
      return;
    }
    selectedId = pad.id;
    armed = true;
    centered = false;
    select.disabled = true;
    enable.disabled = true;
    status.textContent = "Center sticks and release RB, then hold RB to move.";
  };
  stop.onclick = () => disable("Controller input disabled.");
  window.addEventListener("blur", () => {
    if (armed) disable("Window lost focus; controller input disabled.");
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden && armed) disable("Page hidden; controller disabled.");
  });
  window.addEventListener("pagehide", () => {
    if (armed) disable("Controller input disabled.");
  });
  window.addEventListener("gamepadconnected", devices);
  window.addEventListener("gamepaddisconnected", () => {
    if (armed) disable("Controller disconnected.");
    devices();
  });
  devices();
  status.textContent = "Connect the controller and press a button, then enable input.";

  setInterval(async () => {
    if (!armed || busy) return;
    const pad = navigator.getGamepads()[Number(select.value)];
    if (!pad || !pad.connected || pad.id !== selectedId ||
        pad.mapping !== "standard" || !document.hasFocus()) {
      disable("Controller or window unavailable; input disabled.");
      return;
    }
    const axes = pad.axes.slice(0, 4);
    const held = pad.buttons[5].pressed;
    if (!centered) {
      if (held || axes.some(value => Math.abs(value) > settings.deadzone)) return;
      centered = true;
    }
    busy = true;
    pending = new AbortController();
    const timeout = setTimeout(() => pending.abort(), 500);
    try {
      await sendFrame(axes, held, {signal: pending.signal});
      if (armed) {
        status.textContent = `${pad.id}\nRB: ${held ? "held" : "released"}; ` +
          `raw sticks: ${axes.map(value => value.toFixed(3)).join("  ")}`;
      }
    } catch (error) {
      if (armed) disable(`Input stopped: ${error.message}`);
    } finally {
      clearTimeout(timeout);
      pending = null;
      busy = false;
    }
  }, 50);
})();
