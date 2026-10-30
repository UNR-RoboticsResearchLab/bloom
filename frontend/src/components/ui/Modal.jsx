// Modal.jsx
// Accessible dialog shell shared by the admin dialogs: role="dialog" + aria-modal, labelled by the
// caller's heading, focus moves in (to [data-autofocus] or the first control) and is trapped, focus
// returns to whatever opened it, and Escape / backdrop click close it -- unless `dismissible` is
// false, which is for steps where closing would lose something (e.g. a secret shown only once).

import React, { useEffect, useRef } from "react";

const FOCUSABLE =
  'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), summary, [tabindex]:not([tabindex="-1"])';

// Controls inside a closed <details> can't be focused (except its <summary>), so Tab must skip them.
function isFocusable(el) {
  const details = el.closest("details");
  if (!details || details.open) return true;
  return el.tagName === "SUMMARY" && el.parentElement === details;
}

export default function Modal({
  titleId,
  descriptionId,
  onClose,
  dismissible = true,
  widthClass = "max-w-lg",
  children,
}) {
  const dialogRef = useRef(null);
  const openerRef = useRef(document.activeElement);

  useEffect(() => {
    const dialog = dialogRef.current;
    const target = dialog.querySelector("[data-autofocus]") || dialog.querySelector(FOCUSABLE) || dialog;
    target.focus();
    const opener = openerRef.current;
    return () => opener?.focus?.();
  }, []);

  function handleKeyDown(e) {
    if (e.key === "Escape" && dismissible) {
      e.stopPropagation();
      onClose();
      return;
    }
    if (e.key !== "Tab") return;
    const items = [...dialogRef.current.querySelectorAll(FOCUSABLE)].filter(isFocusable);
    if (items.length === 0) {
      e.preventDefault();
      return;
    }
    const first = items[0];
    const last = items[items.length - 1];
    if (e.shiftKey && (document.activeElement === first || document.activeElement === dialogRef.current)) {
      e.preventDefault();
      last.focus();
    } else if (!e.shiftKey && document.activeElement === last) {
      e.preventDefault();
      first.focus();
    }
  }

  return (
    <div className="fixed inset-0 z-40 flex items-center justify-center p-4" onKeyDown={handleKeyDown}>
      <div className="absolute inset-0 bg-black/40" aria-hidden="true" onClick={() => dismissible && onClose()} />
      <div
        ref={dialogRef}
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        aria-describedby={descriptionId}
        tabIndex={-1}
        className={`relative z-50 max-h-[90vh] w-full ${widthClass} overflow-y-auto rounded-xl bg-white shadow-xl focus:outline-none`}
      >
        {children}
      </div>
    </div>
  );
}
