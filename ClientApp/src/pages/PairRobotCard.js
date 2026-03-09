import React, { useState } from "react";

import { useApiClient } from "../context/ApiClientContext";

export function PairRobotCard() {

    const [robotCode, setRobotCode] = useState("");

    const api = useApiClient();

    function handleSubmit(e) {
        e.preventDefault();
        if(!robotCode) return;
        // Call API to get session ID from robot code

        const sessionId = null;
        
        api.getSessionIdFromRobotCode(robotCode)
            .then((res) => {
                sessionId = res?.sessionId;
                if (sessionId) {
                    // Store session ID in state or context as needed
                    console.log("Session ID:", sessionId);
                    // Optionally navigate to the session page
                    // navigate(`/session/${sessionId}`);
                } else {
                    console.error("No session ID returned from server");
                }
            });
        

        
        
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
        <p className="text-lg font-semibold text-gray-900">Pair Robot</p>
        <p className="mt-1 text-sm text-gray-500">Pair a robot to a session.</p>
      </div>

      <form onSubmit={handleSubmit} className="px-6 py-5">
        <div className="space-y-4">
          <div>
            <label className="text-sm font-medium text-gray-700">Robot Code</label>
            <input
              value={robotCode}
              onChange={(e) => setRobotCode(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Enter robot code"
            />
          </div>
        </div>

        <div className="mt-6 border-t border-gray-200 pt-5">
          <div className="flex justify-end gap-3">
            <button
              type="button"
              onClick={onCancel}
              className="rounded-md border border-gray-300 px-4 py-2 text-sm font-medium text-white shadow-sm bg-red-500 hover:bg-red-400"
            >Cancel</button>

            <button
              type="submit"
              className="rounded-md border border-gray-900 px-4 py-2 text-sm font-medium text-white shadow-sm bg-indigo-600 hover:bg-indigo-400 "
            >
              Pair Robot
            </button>
          </div>
        </div>
      </form>
    </div>
  );
}