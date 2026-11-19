/* Only the page Enable/Disable buttons change operator enable state. */
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
  document.querySelector("#gamepad-disabled").hidden = true;
  panel.hidden = false;
  const local = settings.source === "local";
  const mode = settings.simulation ? "Simulation" :
    settings.actuator_transport === "teensy_usb" ?
      "Real mode: Teensy motor output" : "Real mode: numeric previews";
  document.querySelector("#gamepad-mode").textContent =
    `${mode}; ${settings.vehicle}; ${settings.max_effort * 100}% command limit.`;

  async function showPreview() {
    try {
      const data = await request("/api/gamepad");
      const numbers = values => values.map(value => value.toFixed(3)).join("  ");
      const command = data.command ? numbers(data.command) : "waiting / stale";
      const motors = data.preview ? numbers(data.preview) : "waiting / stale";
      preview.textContent = [
        `Movement enabled: ${data.enabled === true ? "yes" : "no"}`,
        `Assistance: ${data.maneuver || "waiting / stale"}`,
        `Body effort [forward, left, up, roll right, turn left]: ${command}`,
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
  setInterval(showPreview, 200);
  showPreview();
  document.querySelector("#gamepad-browser").hidden = local;
  if (!local && (!navigator.getGamepads || !window.isSecureContext)) {
    status.textContent = "Open this page on http://127.0.0.1 through SSH " +
      "using a browser with Gamepad support.";
    enable.disabled = true;
    return;
  }
  const session = crypto.randomUUID();
  let sequence = 0;
  let enabled = false;
  let disablePending = false;
  let busy = false;
  let pending;
  let selectedId;
  stop.disabled = true;

  function devices() {
    if (local) return;
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

  enable.onclick = () => {
    devices();
    if (!local) {
      const pad = navigator.getGamepads()[Number(select.value)];
      if (!pad || pad.mapping !== "standard" || pad.axes.length < 4 ||
          pad.buttons.length < 10) {
        status.textContent = "Connect a standard controller and press a button.";
        return;
      }
      selectedId = pad.id;
    }
    enabled = true;
    disablePending = false;
    select.disabled = true;
    enable.disabled = true;
    stop.disabled = false;
    status.textContent = "Input enabled. Click Disable to stop.";
  };
  stop.onclick = () => {
    enabled = false;
    disablePending = true;
    enable.disabled = false;
    select.disabled = false;
    stop.disabled = true;
    if (pending) pending.abort();
    status.textContent = "Disabling controller input...";
    sendInput();
  };
  window.addEventListener("gamepadconnected", devices);
  window.addEventListener("gamepaddisconnected", devices);
  devices();
  status.textContent = "Click Enable controller input to start.";

  async function controllerInput(pad, signal) {
    const firefoxMac = /Firefox\//.test(navigator.userAgent) &&
      /Mac/.test(navigator.platform);
    if (firefoxMac && /^045e-(02fd|028e)-/i.test(pad.id)) {
      // Gecko drops this model's trigger axes before exposing the gamepad.
      const path = settings.native_controller ? "/api/controller" :
        "http://127.0.0.1:8767/api/controller";
      let frame;
      try {
        const response = await fetch(path, {signal, cache: "no-store"});
        if (!response.ok) throw Error("Controller reader unavailable");
        frame = await response.json();
      } catch (error) {
        if (signal.aborted) throw error;
        throw Error("Start the Mac controller reader on your Mac: " +
          "ros2 run robosub_bringup mac_gamepad");
      }
      const deviceId = id => id.toLowerCase().split("-").slice(0, 2).join("-");
      const inputs = frame.controllers || [frame.controller].filter(Boolean);
      const matches = inputs.filter(
        item => deviceId(item.id) === deviceId(pad.id));
      if (matches.length > 1) {
        throw Error("More than one Mac controller of this model is connected");
      }
      const input = matches[0];
      if (!input) {
        throw Error(frame.error || "Waiting for the Mac Xbox controller");
      }
      return {axes: input.axes, buttons: input.buttons};
    }
    return {
      axes: [...pad.axes.slice(0, 4),
        pad.buttons[6].value, pad.buttons[7].value],
      buttons: [0, 1, 2, 3, 4, 5, 8, 9].map(
        index => pad.buttons[index].pressed ? 1 : 0),
    };
  }

  async function sendInput() {
    if ((!enabled && !disablePending) || busy) return;
    const requestedEnabled = enabled;
    busy = true;
    pending = new AbortController();
    const timeout = setTimeout(() => pending.abort(), 500);
    try {
      let axes = [0, 0, 0, 0, 0, 0];
      let buttons = Array(8).fill(0);
      let inputStatus = "Input enabled on the Linux controller.";
      if (enabled && !local) {
        const pad = navigator.getGamepads()[Number(select.value)];
        if (pad && pad.connected && pad.id === selectedId &&
            pad.mapping === "standard") {
          ({axes, buttons} = await controllerInput(pad, pending.signal));
          inputStatus = `${pad.id}\nSticks + LT/RT: ` +
            axes.map(value => value.toFixed(3)).join("  ") +
            `\nA/B/X/Y/LB/RB/Select/Start: ${buttons.join("  ")}`;
        } else {
          inputStatus = "Waiting for controller; input remains enabled.";
        }
      }
      if (requestedEnabled !== enabled) return;
      await request("/api/gamepad", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-Driver-Station-Token": token,
        },
        body: JSON.stringify({session, sequence: sequence++, axes, buttons,
          enabled: requestedEnabled}),
        signal: pending.signal,
      });
      if (!requestedEnabled) disablePending = false;
      status.textContent = enabled ? inputStatus :
        disablePending ? "Disabling controller input..." : "Input disabled.";
    } catch (error) {
      status.textContent = `Retrying: ${error.message}. ` +
        (enabled ? "Input remains enabled." : "Disable pending acknowledgment.");
    } finally {
      clearTimeout(timeout);
      pending = null;
      busy = false;
    }
  }
  setInterval(sendInput, 50);
})();
