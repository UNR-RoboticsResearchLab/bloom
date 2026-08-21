// src/utils/id.js
// Generates a locally-unique id, used only for React keys / local draft state --
// never persisted or sent to the server.
//
// crypto.randomUUID() only exists in "secure contexts" (HTTPS or localhost), so it's
// undefined when the dev server is opened over plain HTTP via a LAN IP (e.g.
// http://192.168.1.147:3000). crypto.getRandomValues() has no such restriction, so it's
// used to build an equivalent v4 UUID when randomUUID isn't available. Math.random is a
// last-resort fallback for environments with no crypto object at all.
export function randomId() {
  if (typeof crypto !== "undefined" && typeof crypto.randomUUID === "function") {
    return crypto.randomUUID();
  }

  if (typeof crypto !== "undefined" && typeof crypto.getRandomValues === "function") {
    const bytes = crypto.getRandomValues(new Uint8Array(16));
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    const hex = Array.from(bytes, (b) => b.toString(16).padStart(2, "0"));
    return `${hex.slice(0, 4).join("")}-${hex.slice(4, 6).join("")}-${hex.slice(6, 8).join("")}-${hex.slice(8, 10).join("")}-${hex.slice(10, 16).join("")}`;
  }

  return `id-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
}
