/* Passive ROS diagnostics; this view never enables controller input. */
(() => {
  const body = document.querySelector("#diagnostics-rows");
  const status = document.querySelector("#diagnostics-status");
  const levels = ["OK", "Warning", "Error", "Stale / unknown"];
  let busy = false;
  async function refresh() {
    if (busy) return;
    busy = true;
    try {
      const data = await request("/api/diagnostics", {
        signal: AbortSignal.timeout(1500),
      });
      const stale = data.age_s === null || data.age_s > 2;
      status.textContent = stale ? "No fresh ROS hardware diagnostics." :
        "Receiving ROS diagnostics. Allow 15 seconds for motor discovery.";
      body.replaceChildren();
      for (const item of data.rows) {
        const row = document.createElement("tr");
        for (const text of [item.name, levels[stale ? 3 : item.level],
          item.message]) {
          const cell = document.createElement("td");
          cell.textContent = text;
          row.append(cell);
        }
        const details = document.createElement("details");
        const summary = document.createElement("summary");
        summary.textContent = "Values";
        const values = document.createElement("pre");
        values.textContent = Object.entries(item.values).map(
          ([key, value]) => `${key}: ${value}`).join("\n");
        details.append(summary, values);
        row.lastChild.append(details);
        body.append(row);
      }
    } catch (error) {
      status.textContent = `Diagnostics connection: ${error.message}`;
      body.replaceChildren();
    } finally {
      busy = false;
    }
  }
  setInterval(refresh, 500);
  refresh();
})();
