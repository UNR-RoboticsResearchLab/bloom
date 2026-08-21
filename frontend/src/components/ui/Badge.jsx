// Badge.jsx
// Shared pill/tag used for card metadata (lesson type, visibility, etc).

import React from "react";

export default function Badge({ color = "bg-gray-100 text-gray-600", className = "", children }) {
  return (
    <span className={["rounded-full px-2.5 py-0.5 text-xs font-medium", color, className].filter(Boolean).join(" ")}>
      {children}
    </span>
  );
}
