import React from "react";

export default function SelectStudentCard({ students = [], onSelect, onCancel }) {
  return (
    <div className="w-full max-w-xl rounded-lg border border-gray-300 bg-white shadow-sm">
      
      {/* Header */}
      <div className="border-b border-gray-300 px-6 py-4">
        <p className="text-lg font-semibold text-gray-900">Select Student</p>
        <p className="mt-1 text-sm text-gray-500">
          Choose a student for this lesson
        </p>
      </div>

      {/* Scrollable list */}
      <div className="max-h-80 overflow-y-auto px-6 py-4">
        <div className="space-y-2">

          {students.map((student) => (
            <button
              key={student.id}
              onClick={() => onSelect(student)}
              className="w-full rounded-md border border-gray-300 px-4 py-3 text-left hover:bg-gray-100 transition"
            >
              <p className="font-medium text-gray-900">
                {student.firstName} {student.lastName}
              </p>
              <p className="text-sm text-gray-500">
                Level: {student.level}
              </p>
            </button>
          ))}

        </div>
      </div>

      {/* Footer */}
      <div className="border-t border-gray-200 px-6 py-4 flex justify-end">
        <button
          onClick={onCancel}
          className="rounded-md border border-gray-300 px-4 py-2 text-sm font-medium text-white shadow-sm bg-red-500 hover:bg-red-400"
        >
          Cancel
        </button>
      </div>

    </div>
  );
}