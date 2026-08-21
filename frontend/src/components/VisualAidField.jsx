// VisualAidField.jsx
// Reusable image-upload-and-list control backed by the same encoding LessonStep
// (and StepInteraction) use for image fields: a single filename/URL string, or
// a JSON array of them when there's more than one.

import React, { useState } from "react";

export function parseVisualAidList(value) {
  if (!value) return [];
  try {
    const parsed = JSON.parse(value);
    return Array.isArray(parsed) ? parsed : [String(parsed)];
  } catch {
    return [value];
  }
}

export function serializeVisualAidList(list) {
  if (list.length === 0) return "";
  if (list.length === 1) return list[0];
  return JSON.stringify(list);
}

export default function VisualAidField({ value, onChange, max = 4, api }) {
  const [uploading, setUploading] = useState(false);
  const [uploadError, setUploadError] = useState("");

  const list = parseVisualAidList(value);

  async function handleImageUpload(e) {
    const files = Array.from(e.target.files ?? []);
    e.target.value = "";
    if (files.length === 0) return;

    const room = max - list.length;
    if (room <= 0) {
      setUploadError(`Already has the maximum of ${max} image${max === 1 ? "" : "s"}.`);
      return;
    }
    const toUpload = files.slice(0, room);
    if (files.length > toUpload.length) {
      setUploadError(`Only ${max} image${max === 1 ? "" : "s"} allowed — uploaded the first ${toUpload.length}.`);
    } else {
      setUploadError("");
    }

    setUploading(true);
    try {
      const uploaded = await Promise.all(toUpload.map((file) => api.uploadVisualAid(file)));
      const nextList = [...list, ...uploaded.map((r) => r.url)];
      onChange(serializeVisualAidList(nextList));
    } catch (error) {
      setUploadError(error.message || "Upload failed.");
    } finally {
      setUploading(false);
    }
  }

  function removeEntry(index) {
    onChange(serializeVisualAidList(list.filter((_, i) => i !== index)));
  }

  return (
    <div className="mt-2 space-y-2">
      {list.map((url, i) => (
        <div key={i} className="flex items-center gap-2">
          <span className="flex-1 truncate rounded-md border border-gray-200 bg-gray-50 px-3 py-1.5 text-sm text-gray-700">
            {url}
          </span>
          <button
            type="button"
            onClick={() => removeEntry(i)}
            className="text-xs text-red-400 hover:text-red-600"
          >
            Remove
          </button>
        </div>
      ))}
      {list.length < max && (
        <label className="inline-flex cursor-pointer items-center gap-2 rounded-md border border-dashed border-indigo-300 px-3 py-1.5 text-sm text-indigo-600 hover:border-indigo-400 hover:bg-indigo-50">
          {uploading ? "Uploading…" : "+ Upload image"}
          <input
            type="file"
            accept="image/*"
            multiple={max > 1}
            className="sr-only"
            onChange={handleImageUpload}
            disabled={uploading}
          />
        </label>
      )}
      {uploadError && <p className="text-xs text-red-500">{uploadError}</p>}
    </div>
  );
}
