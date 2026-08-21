// Card.jsx
// Shared card container — the visual base for every "browse this list of
// things" card in the app (lessons, students, etc). Extracted so every
// listing page/dashboard renders the same rounded/border/shadow/hover
// treatment instead of each screen hand-rolling its own variant.

import React from "react";

export default function Card({
  as: Component = "div",
  interactive = false,
  accentClassName = "",
  selected = false,
  className = "",
  children,
  ...rest
}) {
  const classes = [
    Component === "button" ? "w-full text-left" : "",
    "rounded-xl border p-5 transition-all",
    selected ? "border-indigo-300 bg-indigo-50" : "border-gray-200 bg-white",
    accentClassName ? `border-l-4 ${accentClassName}` : "",
    "shadow-sm",
    interactive
      ? "hover:shadow-md group focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1"
      : "",
    className,
  ]
    .filter(Boolean)
    .join(" ");

  return (
    <Component className={classes} {...rest}>
      {children}
    </Component>
  );
}
