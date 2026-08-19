import React, { createContext, useCallback, useContext, useEffect, useState } from "react";
import { useApiClient } from "./ApiClientContext";

const RobotPairingContext = createContext(null);

const SESSION_ID_KEY = "pairedSessionId";
const ROBOT_CODE_KEY = "pairedRobotCode";

export const RobotPairingProvider = ({ children }) => {
  const api = useApiClient();
  const [sessionId, setSessionId] = useState(() => localStorage.getItem(SESSION_ID_KEY) || "");
  const [robotCode, setRobotCode] = useState(() => localStorage.getItem(ROBOT_CODE_KEY) || "");

  // Keep pairing state in sync across tabs — pairing/unpairing in one tab
  // should be reflected everywhere without requiring a reload.
  useEffect(() => {
    function handleStorage(e) {
      if (e.key === SESSION_ID_KEY) setSessionId(e.newValue || "");
      if (e.key === ROBOT_CODE_KEY) setRobotCode(e.newValue || "");
    }
    window.addEventListener("storage", handleStorage);
    return () => window.removeEventListener("storage", handleStorage);
  }, []);

  const pair = useCallback(
    async (code) => {
      const trimmed = code.trim();
      // joinSessionByCode (not getSessionIdFromRobotCode) -- this needs to
      // actually claim the session (set its UserId) so the session reflects
      // as paired for anything polling it, e.g. the robot's own pairing-code
      // display, which otherwise never sees userId set and never clears.
      const res = await api.joinSessionByCode(trimmed);
      const returnedSessionId = res?.id ?? res?.sessionId;
      if (!returnedSessionId) {
        throw new Error("No session ID returned from server.");
      }

      localStorage.setItem(SESSION_ID_KEY, returnedSessionId);
      localStorage.setItem(ROBOT_CODE_KEY, trimmed);
      setSessionId(returnedSessionId);
      setRobotCode(trimmed);
      return returnedSessionId;
    },
    [api]
  );

  // Clears local pairing state without calling the server — for callers that
  // have already ended/cleaned up the session through a different endpoint
  // (e.g. LessonView ending the whole lesson session via api.endSession).
  const clearLocal = useCallback(() => {
    localStorage.removeItem(SESSION_ID_KEY);
    localStorage.removeItem(ROBOT_CODE_KEY);
    setSessionId("");
    setRobotCode("");
  }, []);

  const unpair = useCallback(async () => {
    if (sessionId) {
      try {
        await api.unpairRobot(sessionId);
      } catch (err) {
        // Best-effort — still clear local state even if the server call fails
        // (e.g. session already expired server-side).
        console.warn("Failed to unpair robot server-side:", err);
      }
    }
    clearLocal();
  }, [api, sessionId, clearLocal]);

  const value = {
    sessionId: sessionId || null,
    robotCode: robotCode || null,
    isPaired: !!sessionId,
    pair,
    unpair,
    clearLocal,
  };

  return <RobotPairingContext.Provider value={value}>{children}</RobotPairingContext.Provider>;
};

export const useRobotPairing = () => {
  const context = useContext(RobotPairingContext);
  if (!context) {
    throw new Error("useRobotPairing must be used within a RobotPairingProvider");
  }
  return context;
};
