// ToggleSwitch.jsx
// Shared on/off switch control (role="switch") used across the lesson builder
// wherever a boolean setting needs a visible toggle instead of a checkbox.

import React from "react";

export default function ToggleSwitch({ checked, onChange, ariaLabel, title }) {
  return (
    <button
      type="button"
      role="switch"
      aria-checked={checked}
      aria-label={ariaLabel}
      title={title}
      onClick={() => onChange(!checked)}
      className={`relative inline-flex h-6 w-11 shrink-0 items-center rounded-full border-0 p-0 transition-colors ${
        checked ? "bg-indigo-600" : "bg-gray-200"
      }`}
    >
      <span
        className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
          checked ? "translate-x-6" : "translate-x-1"
        }`}
      />
    </button>
  );
}
