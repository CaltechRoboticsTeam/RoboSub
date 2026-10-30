/* The page Enable/Disable buttons are the only movement controls. */
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
          pad.buttons.length < 8) {
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
    status.textContent = "Input enabled. RB is unused. Click Disable to stop.";
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
  status.textContent = "Click Enable controller input to start. RB is unused.";

  async function sendInput() {
    if ((!enabled && !disablePending) || busy) return;
    const requestedEnabled = enabled;
    let axes = [0, 0, 0, 0, 0, 0];
    let inputStatus = "Input enabled on the Linux controller.";
    if (enabled && !local) {
      const pad = navigator.getGamepads()[Number(select.value)];
      if (pad && pad.connected && pad.id === selectedId &&
          pad.mapping === "standard") {
        axes = [...pad.axes.slice(0, 4),
          pad.buttons[6].value, pad.buttons[7].value];
        inputStatus = `${pad.id}\nSticks + LT/RT: ` +
          axes.map(value => value.toFixed(3)).join("  ");
      } else {
        inputStatus = "Waiting for controller; input remains enabled.";
      }
    }
    busy = true;
    pending = new AbortController();
    const timeout = setTimeout(() => pending.abort(), 500);
    try {
      await request("/api/gamepad", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-Tuning-Token": token,
        },
        body: JSON.stringify({session, sequence: sequence++, axes,
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
