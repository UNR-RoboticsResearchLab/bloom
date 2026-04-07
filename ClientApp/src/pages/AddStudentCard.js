import React, { useState } from "react";

export default function AddStudentCard() {
    const [firstName, setFirstName] = useState("");
    const [lastName, setLastName] = useState("");
    const [level, setLevel] = useState("");

    function handleSubmit(e) {
        e.preventDefault();
        if(!firstName || !lastName || !level) return;
    }

    function onCancel(){
        return;
    }

    function onAdd(){
        return;
    }

        return (
    <div className="w-full max-w-xl rounded-lg border border-gray-300 bg-white shadow-sm">
      <div className="border-b border-gray-300 px-6 py-4">
        <p className="text-lg font-semibold text-gray-900">Add Student</p>
        <p className="mt-1 text-sm text-gray-500">Create a student profile</p>
      </div>

      <form onSubmit={handleSubmit} className="px-6 py-5">
        <div className="space-y-4">
          <div>
            <label className="text-sm font-medium text-gray-700">First Name</label>
            <input
              value={firstName}
              onChange={(e) => setFirstName(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Enter first name"
            />
          </div>

          <div>
            <label className="text-sm font-medium text-gray-700">Last Name</label>
            <input
              value={lastName}
              onChange={(e) => setLastName(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Enter last name"
            />
          </div>

          <div>
            <label className="text-sm font-medium text-gray-700">Level</label>
            <input
              value={level}
              onChange={(e) => setLevel(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Enter level"
            />
          </div>
        </div>

        <div className="mt-6 border-t border-gray-200 pt-5">
          <div className="flex justify-end gap-3">
            <button
              type="button"
              onClick={onCancel}
              className="rounded-md border border-gray-300 px-4 py-2 text-sm font-medium text-white shadow-sm bg-red-500 hover:bg-red-400"
            >
              Cancel
            </button>

            <button
              type="submit"
              className="rounded-md border border-gray-900 px-4 py-2 text-sm font-medium text-white shadow-sm bg-indigo-600 hover:bg-indigo-400 "
            >
              Add Student
            </button>
          </div>
        </div>
      </form>
    </div>
  );
}