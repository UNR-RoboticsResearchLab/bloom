import { useApiClient } from "../context/ApiClientContext";
import { useRobotPairing } from "../context/RobotPairingContext";
import { useLocation, useNavigate } from "react-router-dom";
import { useCallback, useEffect, useRef, useState } from "react";

export default function LessonView() {
    const location = useLocation();
    const navigate = useNavigate();
    const api = useApiClient();
    const { sessionId: pairedSessionId, clearLocal } = useRobotPairing();
    const hasStartedRef = useRef(false);

    const { lesson, student } = location.state || {};
    const lessonId = lesson?.id ?? lesson?.Id;

    const [activeSessionId, setActiveSessionId] = useState(pairedSessionId);

    const [noteText, setNoteText] = useState("");
    const [stepInput, setStepInput] = useState("");
    const [isSendingStepCommand, setIsSendingStepCommand] = useState(false);
    const [step, setStep] = useState([]);
    const [isLoadingSteps, setIsLoadingSteps] = useState(false);
    const [isLoadingHistory, setIsLoadingHistory] = useState(false);
    const [isEndingLesson, setIsEndingLesson] = useState(false);
    const [isPaused, setIsPaused] = useState(false);
    const [isTogglingPause, setIsTogglingPause] = useState(false);

    const [conversation, setConversation] = useState([]);

    const lessonTitle = lesson?.title ?? lesson?.Title ?? "Untitled Lesson";
    const studentName =
        student?.fullName ??
        student?.studentName ??
        student?.name ??
        "Unknown Student";

    const formatTime = (value) => {
        if (!value) {
            return new Date().toLocaleTimeString([], {
                hour: "numeric",
                minute: "2-digit",
            });
        }

        const date = new Date(value);

        if (Number.isNaN(date.getTime())) {
            return new Date().toLocaleTimeString([], {
                hour: "numeric",
                minute: "2-digit",
            });
        }

        return date.toLocaleTimeString([], {
            hour: "numeric",
            minute: "2-digit",
        });
    };

    // const mapHistoryItem = useCallback((item, index) => {
    //     let type = "robot";

    //     if (item.source === "Lesson") type = "student";
    //     if (item.source === "SLP") type = "note";

    //     return {
    //         id: item.id ?? `event-${index}`,
    //         type,
    //         text: item.message,
    //         ts: formatTime(item.timestamp),
    //     };
    // }, []);

    const loadLessonSteps = useCallback(async () => {
        if (!lessonId) return;

        try {
            setIsLoadingSteps(true);

            const lessonData = await api.getLesson(lessonId);
            console.log("Lesson details:", lessonData);

            const mappedSteps = (lessonData?.steps ?? lessonData?.Steps ?? [])
                .sort(
                    (a, b) =>
                        (a.stepOrder ?? a.StepOrder ?? 0) -
                        (b.stepOrder ?? b.StepOrder ?? 0)
                )
                .map((item) => ({
                    id: item.id ?? item.Id ?? item.stepOrder ?? item.StepOrder,
                    order: item.stepOrder ?? item.StepOrder,
                    type: item.type ?? item.Type,
                    title: `Step ${item.stepOrder ?? item.StepOrder}: ${item.type ?? item.Type}`,
                    text: item.script ?? item.Script ?? "No script available.",
                }));

            setStep(mappedSteps);
        } catch (error) {
            console.error("Failed to load lesson steps:", error);
            setStep([]);
        } finally {
            setIsLoadingSteps(false);
        }
    }, [api, lessonId]);

    const loadSessionHistory = useCallback(async (sessionIdToUse = activeSessionId) => {
        if (!sessionIdToUse) return;

        try {
            setIsLoadingHistory(true);

            const interactions = await api.getLessonInteractions(sessionIdToUse);
            console.log("LESSON INTERACTIONS:", interactions);

            const interactionArray = Array.isArray(interactions) ? interactions : [];

            const mapped = interactionArray
                .map((item, index) => {
                    const interactionType = String(item.interactionType ?? "").toLowerCase();

                    return {
                        id: item.id ?? `interaction-${index}`,
                        type:
                            interactionType === "response"
                                ? "student"
                                : interactionType === "note" || interactionType === "slpfeedback"
                                ? "note"
                                : "robot",
                        text: item.studentResponse || item.dialogTurn || "",
                        ts: formatTime(item.timestamp),
                    };
                })
                .filter((item) => item.text);

            setConversation(mapped);
        } catch (error) {
            console.error("Failed to load lesson interactions:", error);
        } finally {
            setIsLoadingHistory(false);
        }
    }, [activeSessionId, api]);

    const getValidSessionId = useCallback(() => {
        if (!pairedSessionId) {
            setActiveSessionId(null);
            return null;
        }
        setActiveSessionId(pairedSessionId);
        return pairedSessionId;
    }, [pairedSessionId]);

    useEffect(() => {
        async function startLesson() {
            if (!lesson || !student) {
                navigate("/lessons");
                return;
            }

            if (!lessonId) {
                console.error("Missing lessonId");
                return;
            }

            if (hasStartedRef.current) {
                return;
            }

            hasStartedRef.current = true;

            try {
                const sessionIdToUse = getValidSessionId();

                if (!sessionIdToUse) {
                    console.error("Missing sessionId");
                    return;
                }

                console.log("Starting lesson session with:", {
                    lessonId,
                    sessionId: sessionIdToUse,
                    student,
                });

                const res = await api.startLessonSession(sessionIdToUse, lessonId, student?.id ?? student?.studentId ?? sessionIdToUse);
                console.log("Lesson session started:", res);

                await loadLessonSteps();
                await loadSessionHistory(sessionIdToUse);
            } catch (error) {
                console.error("Failed to start lesson session:", error);
                clearLocal();
                setActiveSessionId(null);
            }
        }

        startLesson();
    }, [
        lesson,
        student,
        lessonId,
        navigate,
        api,
        clearLocal,
        getValidSessionId,
        loadLessonSteps,
        loadSessionHistory,
    ]);

    useEffect(() => {
        if (!activeSessionId) return;

        const intervalId = setInterval(() => {
            loadSessionHistory(activeSessionId);
        }, 3000);

        return () => clearInterval(intervalId);
    }, [activeSessionId, loadSessionHistory]);

    async function addNote(e) {
        e.preventDefault();

        const trimmed = noteText.trim();
        if (!trimmed || !activeSessionId) return;

        const optimisticNote = {
            id: `note-${Date.now()}`,
            type: "note",
            text: trimmed,
            ts: formatTime(),
        };

        try {
            setConversation((prev) => [...prev, optimisticNote]);
            setNoteText("");

            await api.addNoteToSession(activeSessionId, trimmed);
            await loadSessionHistory(activeSessionId);
        } catch (error) {
            console.error("Failed to add note:", error);
        }
    }

    async function handleBackStep() {
        if (!activeSessionId || isSendingStepCommand || isPaused) return;
        try {
            setIsSendingStepCommand(true);
            await api.replayStep(activeSessionId);
        } catch (error) {
            console.error("Failed to queue replay command:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleForwardStep() {
        if (!activeSessionId || isSendingStepCommand || isPaused) return;
        try {
            setIsSendingStepCommand(true);
            await api.skipStep(activeSessionId);
        } catch (error) {
            console.error("Failed to queue skip command:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleSkipToStep() {
        const trimmed = stepInput.trim();
        if (!trimmed || !activeSessionId || isSendingStepCommand || isPaused) return;

        const targetStep = Number(trimmed);
        if (!Number.isInteger(targetStep) || targetStep < 1) {
            console.error("Invalid step number:", trimmed);
            return;
        }

        try {
            setIsSendingStepCommand(true);
            await api.setStep(activeSessionId, targetStep);
            setStepInput("");
        } catch (error) {
            console.error("Failed to queue set step command:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleStepClick(stepNumber) {
        if (!activeSessionId || isSendingStepCommand || isPaused) return;
        try {
            setIsSendingStepCommand(true);
            await api.setStep(activeSessionId, stepNumber);
        } catch (error) {
            console.error("Failed to jump to step:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleRestart() {
        if (!activeSessionId || isSendingStepCommand || isPaused) return;
        try {
            setIsSendingStepCommand(true);
            await api.setStep(activeSessionId, 1);
        } catch (error) {
            console.error("Failed to restart lesson:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleTogglePause() {
        if (!activeSessionId || isTogglingPause) return;
        try {
            setIsTogglingPause(true);
            if (isPaused) {
                await api.resumeLesson(activeSessionId);
                setIsPaused(false);
            } else {
                await api.pauseLesson(activeSessionId);
                setIsPaused(true);
            }
        } catch (error) {
            console.error(`Failed to ${isPaused ? "resume" : "pause"} lesson:`, error);
        } finally {
            setIsTogglingPause(false);
        }
    }

    async function handleEndLesson() {
        if (!activeSessionId || isEndingLesson) return;

        setIsEndingLesson(true);
        try {
            await api.stopLesson(activeSessionId);
        } catch (e) {
            console.warn("Failed to stop lesson (may not be active):", e);
        }
        try {
            await api.endSession(activeSessionId);
        } catch (e) {
            // Demo mode has no cookie auth — server session will be cleaned up by the
            // robot's inactivity timer. Don't block navigation on a 401 here.
            console.warn("Failed to end session server-side:", e);
        }
        clearLocal();
        setIsEndingLesson(false);
        navigate("/lessons");
    }

    function renderMessage(item) {
        if (item.type === "robot") {
            return (
                <div key={item.id} className="flex flex-col items-start max-w-[75%]">
                    <p className="text-xs font-semibold text-gray-600 mb-1">Robot</p>

                    <div className="rounded-2xl rounded-bl-md bg-gray-200 px-4 py-3 text-sm text-gray-900 shadow-sm">
                        <p>{item.text}</p>
                    </div>

                    <p className="mt-1 text-[11px] text-gray-500">{item.ts}</p>
                </div>
            );
        }

        if (item.type === "student") {
            return (
                <div key={item.id} className="flex flex-col items-end max-w-[75%] ml-auto">
                    <p className="text-xs font-semibold text-gray-600 mb-1">Student</p>

                    <div className="rounded-2xl rounded-br-md bg-green-500 px-4 py-3 text-sm text-white shadow-sm">
                        <p>{item.text}</p>
                    </div>

                    <p className="mt-1 text-[11px] text-gray-500">{item.ts}</p>
                </div>
            );
        }

        return (
            <div key={item.id} className="flex flex-col items-center max-w-[70%] mx-auto">
                <p className="text-xs font-semibold text-gray-600 mb-1">SLP Note</p>

                <div className="rounded-xl border border-yellow-300 bg-yellow-100 px-4 py-3 text-sm text-gray-900 shadow-sm text-center">
                    <p>{item.text}</p>
                </div>

                <p className="mt-1 text-[11px] text-gray-500">{item.ts}</p>
            </div>
        );
    }

    if (!lesson || !student) {
        return null;
    }

    return (
        <div className="mt-4 space-y-4">
            <div className="rounded-2xl border border-gray-200 bg-white p-6 shadow-sm">
                <div className="flex flex-col gap-4 lg:flex-row lg:items-center lg:justify-between">
                    <div>
                        <p className="text-sm font-semibold uppercase tracking-wide text-indigo-600">
                            Lesson In Progress
                        </p>

                        <h1 className="mt-1 text-2xl font-bold text-gray-900">
                            {lessonTitle}
                        </h1>

                        <p className="mt-2 text-sm text-gray-600">
                            Student:{" "}
                            <span className="font-semibold text-gray-900">
                                {studentName}
                            </span>
                        </p>
                    </div>

                    <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
                        <div className="rounded-2xl border border-gray-200 bg-gray-50 px-4 py-3 shadow-sm">
                            <p className="text-xs font-semibold uppercase tracking-wide text-gray-500">
                                Status
                            </p>
                            <p className="mt-1 text-sm font-semibold text-green-600">
                                Active
                            </p>
                        </div>

                        <div className="rounded-2xl border border-gray-200 bg-gray-50 px-4 py-3 shadow-sm">
                            <p className="text-xs font-semibold uppercase tracking-wide text-gray-500">
                                Session
                            </p>
                            <p className="mt-1 text-sm font-semibold text-gray-900">
                                {activeSessionId ? "Connected" : "Missing"}
                            </p>
                        </div>

                        <div className="rounded-2xl border border-gray-200 bg-gray-50 px-4 py-3 shadow-sm">
                            <p className="text-xs font-semibold uppercase tracking-wide text-gray-500">
                                Steps
                            </p>
                            <p className="mt-1 text-sm font-semibold text-gray-900">
                                {step.length}
                            </p>
                        </div>

                        <div className="rounded-2xl border border-gray-200 bg-gray-50 px-4 py-3 shadow-sm">
                            <p className="text-xs font-semibold uppercase tracking-wide text-gray-500">
                                Notes
                            </p>
                            <p className="mt-1 text-sm font-semibold text-gray-900">
                                {conversation.filter((item) => item.type === "note").length}
                            </p>
                        </div>
                    </div>
                </div>
            </div>

            <div className="rounded-lg border p-6 shadow-sm">
                <div className="rounded-lg border p-4 shadow-sm">
                    <div className="flex items-center justify-between gap-2">
                        <h2 className="text-lg font-semibold text-gray-900">
                            Lesson Controls
                            {isPaused && (
                                <span className="ml-2 rounded-full bg-amber-100 px-2 py-0.5 text-xs font-semibold text-amber-800 align-middle">
                                    Paused
                                </span>
                            )}
                        </h2>

                        <button
                            type="button"
                            onClick={handleTogglePause}
                            disabled={isTogglingPause}
                            className={`flex items-center justify-center gap-2 rounded-xl px-4 py-2 text-sm font-semibold shadow-sm transition active:scale-[0.98] disabled:cursor-not-allowed disabled:opacity-60 ${
                                isPaused
                                    ? "bg-green-100 text-green-800 hover:bg-green-200"
                                    : "bg-amber-100 text-amber-800 hover:bg-amber-200"
                            }`}
                        >
                            {isTogglingPause
                                ? (isPaused ? "Resuming..." : "Pausing...")
                                : (isPaused ? "Resume" : "Pause")}
                        </button>

                        <button
                            type="button"
                            onClick={handleRestart}
                            disabled={isSendingStepCommand || isPaused}
                            className="flex items-center justify-center gap-2 rounded-xl bg-yellow-100 px-4 py-2 text-sm font-semibold text-yellow-800 shadow-sm transition hover:bg-yellow-200 hover:shadow-md active:scale-[0.98] disabled:cursor-not-allowed disabled:opacity-60"
                        >
                            <span className="text-lg">&#10226;</span>
                            {isSendingStepCommand ? "Restarting..." : "Restart"}
                        </button>

                        <button
                            type="button"
                            onClick={handleEndLesson}
                            disabled={isEndingLesson}
                            className="rounded-xl bg-red-500 px-4 py-2 text-sm font-semibold text-white shadow-sm hover:bg-red-400 active:scale-[0.98] disabled:cursor-not-allowed disabled:opacity-60"
                        >
                            {isEndingLesson ? "Ending..." : "End Lesson"}
                        </button>
                    </div>

                    <div className="mt-2 h-[200px] overflow-x-auto overflow-y-hidden rounded-2xl border bg-white p-3">
                        <div className="flex gap-3 h-full items-stretch">
                            {isLoadingSteps ? (
                                <div className="flex h-full min-w-full items-center justify-center">
                                    <p className="text-sm text-gray-500">
                                        Loading lesson steps...
                                    </p>
                                </div>
                            ) : step.length === 0 ? (
                                <div className="flex h-full min-w-full items-center justify-center">
                                    <p className="text-sm text-gray-500">
                                        No lesson steps found.
                                    </p>
                                </div>
                            ) : (
                                step.map((item) => (
                                    <div
                                        key={item.id}
                                        onClick={() => !isSendingStepCommand && !isPaused && handleStepClick(item.order)}
                                        className= {`min-w-[180px] max-w-[220px] flex-shrink-0 h-full rounded-lg bg-gray-100 border border-gray-300 p-3 shadow-sm flex flex-col justify-start ${isSendingStepCommand || isPaused ? "opacity-50 cursor-not-allowed" : "cursor-pointer hover:bg-blue-100"}`}
                                    >
                                        <p className="text-sm font-semibold text-gray-900 leading-tight">
                                            {item.title}
                                        </p>

                                        <p className="mt-2 text-xs text-gray-600 leading-snug break-words">
                                            {item.text}
                                        </p>
                                    </div>
                                ))
                            )}
                        </div>
                    </div>

                    <div className="mt-4 grid gap-4 md:grid-cols-[1fr_2fr_1fr] items-stretch">
                        <button
                            type="button"
                            onClick={handleBackStep}
                            disabled={isSendingStepCommand || isPaused}
                            className="flex h-full items-center justify-center gap-2 rounded-2xl border border-gray-200 hover:bg-blue-200 bg-white px-4 py-3 text-sm font-semibold text-gray-700 shadow-sm transition hover:shadow-md disabled:cursor-not-allowed disabled:opacity-60"
                        >
                            <span className="text-lg">{"<"}</span>
                            Back
                        </button>

                        <div className="flex flex-col items-center rounded-2xl border border-gray-200 bg-white p-4 shadow-sm">
                            <p className="mb-2 text-xs font-semibold text-gray-500 uppercase tracking-wide">
                                Jump to Step
                            </p>

                            <input
                                type="number"
                                value={stepInput}
                                onChange={(e) => setStepInput(e.target.value)}
                                placeholder="Step #"
                                disabled={isSendingStepCommand || isPaused}
                                className="w-full rounded-xl border border-gray-300 p-3 text-center text-sm font-medium shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 disabled:cursor-not-allowed disabled:opacity-60"
                            />

                            <button
                                type="button"
                                onClick={handleSkipToStep}
                                disabled={isSendingStepCommand || isPaused}
                                className="mt-3 w-full rounded-xl bg-indigo-600 px-4 py-2.5 text-sm font-semibold text-white shadow-sm transition hover:bg-indigo-500 hover:shadow-md disabled:cursor-not-allowed disabled:opacity-60"
                            >
                                {isSendingStepCommand ? "Sending..." : "Go"}
                            </button>
                        </div>

                        <button
                            type="button"
                            onClick={handleForwardStep}
                            disabled={isSendingStepCommand || isPaused}
                            className="flex h-full items-center justify-center gap-2 rounded-2xl border border-gray-200 hover:bg-blue-200 bg-white px-4 py-3 text-sm font-semibold text-gray-700 shadow-sm transition hover:shadow-md disabled:cursor-not-allowed disabled:opacity-60"
                        >
                            Forward
                            <span className="text-lg">{">"}</span>
                        </button>
                    </div>
                </div>

                <div className="mt-4 rounded-lg border p-4 shadow-sm">
                    <div className="flex items-center justify-between">
                        <h2 className="text-lg font-semibold text-gray-900">
                            Lesson Transcript
                        </h2>

                        {isLoadingHistory && (
                            <p className="text-xs font-semibold text-gray-500">
                                Updating...
                            </p>
                        )}
                    </div>

                    <div className="mt-2 h-[750px] overflow-y-auto rounded-2xl border bg-white p-4 space-y-4">
                        {conversation.length === 0 ? (
                            <p className="text-sm text-gray-500">No activity yet.</p>
                        ) : (
                            conversation.map((item) => renderMessage(item))
                        )}
                    </div>

                    <form onSubmit={addNote} className="mt-4 border-t pt-4">
                        <label className="text-sm font-medium text-gray-700">
                            Add SLP Note
                        </label>

                        <div className="mt-2 flex gap-3">
                            <textarea
                                value={noteText}
                                onChange={(e) => setNoteText(e.target.value)}
                                rows={3}
                                placeholder="Write a note"
                                className="flex-1 rounded-xl border border-gray-300 p-3 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
                            />

                            <button
                                type="submit"
                                className="self-end rounded-xl bg-indigo-600 px-5 py-3 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500"
                            >
                                Add Note
                            </button>
                        </div>
                    </form>
                </div>
            </div>
        </div>
    );
}