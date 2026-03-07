import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";
import { getSession } from "../utils/auth";

export default function AddLessonCard() {
  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [lessonType, setLessonType] = useState("Language");
  const [lessonDescription, setLessonDescription] = useState("");
  const [err, setErr] = useState("");
  const [submitting, setSubmitting] = useState(false);
  const [success, setSuccess] = useState(false);

  const navigate = useNavigate();
  const api = useApiClient();

  function handleSubmit(e) {
    e.preventDefault();
    setErr("");

    if (!title.trim()) {
      setErr("Title is required");
      return;
    }

    if (!lessonDescription.trim()) {
      setErr("Lesson JSON content is required");
      return;
    }

    try {
      JSON.parse(lessonDescription);
    } catch (parseErr) {
      setErr("Lesson JSON is invalid. Please check the syntax.");
      return;
    }

    submitLesson();
  }

  async function submitLesson() {
    setSubmitting(true);
    try {
      const session = getSession();
      if (!session?.id) {
        setErr("Unable to determine user ID. Please sign in again.");
        return;
      }

      const payload = {
        title: title.trim(),
        description: description.trim(),
        lessonType: lessonType === "Language" ? 0 : 1,
        lessonDescription: lessonDescription,
        createdById: session.id,
      };

      const response = await api.createLesson(payload);

      if (response?.message && response.message.includes("successfully")) {
        setSuccess(true);
        setTimeout(() => {
          navigate("/dashboard/admin");
        }, 1500);
      } else {
        setErr(response?.message || "Failed to create lesson");
      }
    } catch (error) {
      console.error("Lesson creation error:", error);
      setErr(error.message || "An error occurred while creating the lesson");
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <div className="w-full max-w-2xl rounded-lg border border-gray-300 bg-white shadow-sm">
      <div className="border-b border-gray-300 px-6 py-4">
        <p className="text-lg font-semibold text-gray-900">Add Lesson</p>
        <p className="mt-1 text-sm text-gray-500">Create a new lesson for the system</p>
      </div>

      <form onSubmit={handleSubmit} className="px-6 py-5">
        <div className="space-y-4">
          {err && <p className="text-sm text-red-600">{err}</p>}
          {success && <p className="text-sm text-green-600">Lesson created successfully! Redirecting...</p>}

          {/* Title */}
          <div>
            <label className="text-sm font-medium text-gray-700">Title *</label>
            <input
              value={title}
              onChange={(e) => setTitle(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Lesson title"
            />
          </div>

          {/* Description */}
          <div>
            <label className="text-sm font-medium text-gray-700">Description</label>
            <textarea
              value={description}
              onChange={(e) => setDescription(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Optional lesson description"
              rows="2"
            />
          </div>

          {/* Lesson Type */}
          <div>
            <label className="text-sm font-medium text-gray-700">Lesson Type *</label>
            <select
              value={lessonType}
              onChange={(e) => setLessonType(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
            >
              <option value="Language">Language</option>
              <option value="SpeechTherapy">Speech Therapy</option>
            </select>
          </div>

          {/* Lesson JSON */}
          <div>
            <label className="text-sm font-medium text-gray-700">Lesson JSON Content *</label>
            <textarea
              value={lessonDescription}
              onChange={(e) => setLessonDescription(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm font-mono focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder='{"id": "lesson_001", "title": "...", "sequence": [...]}'
              rows="8"
            />
            <p className="mt-1 text-xs text-gray-500">Must be valid JSON format</p>
          </div>
        </div>

        {/* Buttons */}
        <div className="mt-6 border-t border-gray-200 pt-5">
          <div className="flex justify-end gap-3">
            <button
              type="button"
              onClick={() => navigate("/dashboard/admin")}
              className="rounded-md border border-gray-300 px-4 py-2 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-50"
            >
              Cancel
            </button>

            <button
              type="submit"
              disabled={submitting}
              className="rounded-md border border-gray-900 px-4 py-2 text-sm font-medium text-white shadow-sm bg-indigo-600 hover:bg-indigo-500 disabled:opacity-50"
            >
              {submitting ? "Creating..." : "Create Lesson"}
            </button>
          </div>
        </div>
      </form>
    </div>
  );
}
