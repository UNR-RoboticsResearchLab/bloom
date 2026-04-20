import { useApiClient } from "../context/ApiClientContext";
import { useLocation, useNavigate } from "react-router-dom";
import { useEffect, useRef, useState } from "react";

export default function LessonView() {
    const location = useLocation();
    const navigate = useNavigate();
    const api = useApiClient();
    const hasStartedRef = useRef(false);

    const sessionId = localStorage.getItem("pairedSessionId");
    const { lesson, student } = location.state || {};
    const lessonId = lesson?.id ?? lesson?.Id;

    const [noteText, setNoteText] = useState("");
    const [stepInput, setStepInput] = useState("");
    const [isSendingStepCommand, setIsSendingStepCommand] = useState(false);



    const [step, setStep] = useState([]);
    const [isLoadingSteps, setIsLoadingSteps] = useState(false);

    async function loadLessonSteps() {
        if (!lessonId) return;

        try {
            setIsLoadingSteps(true);

            const lessonData = await api.getLesson(lessonId);
            console.log("Lesson details:", lessonData);

            const mappedSteps = (lessonData?.steps ?? lessonData?.Steps ?? [])
                .sort((a, b) => (a.stepOrder ?? a.StepOrder ?? 0) - (b.stepOrder ?? b.StepOrder ?? 0))
                .map((item) => ({
                    id: item.id ?? item.Id ?? item.stepOrder ?? item.StepOrder,
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
    }


    // Fake data needs to be replaced with real conversation data from the backend
    const [conversation, setConversation] = useState([
        {
            id: 1,
            type: "robot",
            text: "Hi there! I’m really happy to see you today. Are you ready to practice some words together?",
            ts: "10:00 AM",
        },
        {
            id: 2,
            type: "student",
            text: "Yeah.",
            ts: "10:01 AM",
        },
        {
            id: 3,
            type: "note",
            text: "Student appears engaged and responsive. Maintains attention and makes eye contact.",
            ts: "10:01 AM",
        },

        {
            id: 4,
            type: "robot",
            text: "Great! Let’s start with a word. Listen carefully: rrrred. Can you try saying red?",
            ts: "10:02 AM",
        },
        {
            id: 5,
            type: "student",
            text: "wed.",
            ts: "10:03 AM",
        },
        {
            id: 6,
            type: "note",
            text: "Substitution error observed. /r/ sound replaced with /w/.",
            ts: "10:03 AM",
        },

        {
            id: 7,
            type: "robot",
            text: "Nice try! Let’s try that again. Watch my mouth: rrrred.",
            ts: "10:04 AM",
        },
        {
            id: 8,
            type: "student",
            text: "rrrr... red.",
            ts: "10:05 AM",
        },
        {
            id: 9,
            type: "note",
            text: "Improvement with prompting. Approximate /r/ achieved.",
            ts: "10:05 AM",
        },

        {
            id: 10,
            type: "robot",
            text: "That was really good! Now try saying just the word: red.",
            ts: "10:06 AM",
        },
        {
            id: 11,
            type: "student",
            text: "red.",
            ts: "10:07 AM",
        },
        {
            id: 12,
            type: "note",
            text: "Correct production achieved independently.",
            ts: "10:07 AM",
        },

        {
            id: 13,
            type: "robot",
            text: "Awesome! Now let’s use it in a sentence.",
            ts: "10:08 AM",
        },
        {
            id: 14,
            type: "student",
            text: "I see a red ball.",
            ts: "10:09 AM",
        },
        {
            id: 15,
            type: "note",
            text: "Generalization successful.",
            ts: "10:09 AM",
        },

        {
            id: 16,
            type: "robot",
            text: "That was excellent work today!",
            ts: "10:10 AM",
        },
        {
            id: 17,
            type: "student",
            text: "Yay!",
            ts: "10:10 AM",
        },
        {
            id: 18,
            type: "note",
            text: "Session complete. Student improved with reduced cueing.",
            ts: "10:11 AM",
        }
    ]);

    useEffect(() => {
        async function startLesson() {
            if (!lesson || !student) {
                navigate("/lessons");
                return;
            }

            if (!sessionId || !lessonId) {
                console.error("Missing sessionId or lessonId");
                return;
            }

            if (hasStartedRef.current) {
                return;
            }

            hasStartedRef.current = true;

            try {
                console.log("Starting lesson session with:", {
                    lessonId,
                    sessionId,
                    student,
                });

                const res = await api.startLessonSession(lessonId, sessionId);
                console.log("Lesson session started:", res);
                
                await loadLessonSteps();
            } catch (error) {
                console.error("Failed to start lesson session:", error);
            }
        }

        startLesson();
    }, [lesson, student, lessonId, sessionId, navigate, api]);

    function addNote(e) {
        e.preventDefault();

        const trimmed = noteText.trim();
        if (!trimmed) return;

        const newNote = {
            id: Date.now(),
            type: "note",
            text: trimmed,
            ts: new Date().toLocaleTimeString(),
        };

        setConversation((prev) => [...prev, newNote]);
        setNoteText("");
    }

    async function handleBackStep() {
        if (!sessionId || isSendingStepCommand) return;

        try {
            setIsSendingStepCommand(true);

            const res = await api.request(
                `/api/LessonSession/${sessionId}/lessons/replay`,
                {
                    method: "POST",
                }
            );

            console.log("Replay command queued:", res);
        } catch (error) {
            console.error("Failed to queue replay command:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleForwardStep() {
        if (!sessionId || isSendingStepCommand) return;

        try {
            setIsSendingStepCommand(true);

            const res = await api.request(
                `/api/LessonSession/${sessionId}/lessons/skip`,
                {
                    method: "POST",
                }
            );

            console.log("Skip command queued:", res);
        } catch (error) {
            console.error("Failed to queue skip command:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    async function handleSkipToStep() {
        const trimmed = stepInput.trim();
        if (!trimmed || !sessionId || isSendingStepCommand) return;

        const targetStep = Number(trimmed);

        if (!Number.isInteger(targetStep) || targetStep < 1) {
            console.error("Invalid step number:", trimmed);
            return;
        }

        try {
            setIsSendingStepCommand(true);

            const res = await api.request(
                `/api/LessonSession/${sessionId}/lessons/set-step`,
                {
                    method: "POST",
                    body: JSON.stringify({ targetStep }),
                }
            );

            console.log("Set step command queued:", res);
            setStepInput("");
        } catch (error) {
            console.error("Failed to queue set-step command:", error);
        } finally {
            setIsSendingStepCommand(false);
        }
    }

    function renderMessage(item) {
        if (item.type === "robot") {
            return (
                <div key={item.id} className="flex flex-col items-start max-w-[75%]">
                    <p className="text-xs font-semibold text-gray-600 mb-1">
                        Robot
                    </p>
                    <div className="rounded-2xl rounded-bl-md bg-gray-200 px-4 pt-3 text-sm text-gray-900 shadow-sm">
                        <p>{item.text}</p>
                    </div>
                    <p className="mt-1 text-[11px] text-gray-500">
                        {item.ts}
                    </p>
                </div>
            );
        }

        if (item.type === "student") {
            return (
                <div key={item.id} className="flex flex-col items-end max-w-[75%] ml-auto">
                    <p className="text-xs font-semibold text-gray-600 mb-1">
                        Student
                    </p>
                    <div className="rounded-2xl rounded-br-md bg-green-500 px-4 pt-3 text-sm text-white shadow-sm">
                        <p>{item.text}</p>
                    </div>
                    <p className="mt-1 text-[11px] text-gray-500">
                        {item.ts}
                    </p>
                </div>
            );
        }

        return (
            <div key={item.id} className="flex flex-col items-center max-w-[70%] mx-auto">
                <p className="text-xs font-semibold text-gray-600 mb-1">
                    SLP Note
                </p>
                <div className="rounded-xl border border-yellow-300 bg-yellow-100 px-4 pt-3 text-sm text-gray-900 shadow-sm text-center">
                    <p>{item.text}</p>
                </div>
                <p className="mt-1 text-[11px] text-gray-500">
                    {item.ts}
                </p>
            </div>
        );
    }

    if (!lesson || !student) {
        return null;
    }

    return (
        <div className="mt-4 rounded-lg border p-6 shadow-sm">
            <p className="text-sm font-semibold text-gray-900">
                Lesson In Progress
            </p>

            <div className="mt-4 rounded-lg border p-4 shadow-sm">
                <div className="flex items-center justify-between">
                    <h2 className="text-lg font-semibold text-gray-900">Lesson Controls</h2>
                </div>

                <div className="mt-2 h-[200px] overflow-x-auto overflow-y-hidden rounded-2xl border bg-white p-3">
                    <div className="flex gap-3 h-full items-stretch">
                        {isLoadingSteps ? (
                            <div className="flex h-full min-w-full items-center justify-center">
                                <p className="text-sm text-gray-500">Loading lesson steps...</p>
                            </div>
                        ) : step.length === 0 ? (
                            <div className="flex h-full min-w-full items-center justify-center">
                                <p className="text-sm text-gray-500">No lesson steps found.</p>
                            </div>
                        ) : (
                            step.map((item) => (
                                <div
                                    key={item.id}
                                    className="min-w-[180px] max-w-[220px] flex-shrink-0 h-full rounded-lg bg-gray-100 border border-gray-300 p-3 shadow-sm flex flex-col justify-start"
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
                        disabled={isSendingStepCommand}
                        className="flex h-full items-center justify-center gap-2 rounded-2xl border border-gray-200  hover:bg-blue-200 bg-white px-4 py-3 text-sm font-semibold text-gray-700 shadow-sm transition hover:shadow-md disabled:cursor-not-allowed disabled:opacity-60"
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
                            disabled={isSendingStepCommand}
                            className="w-full rounded-xl border border-gray-300 p-3 text-center text-sm font-medium shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 disabled:cursor-not-allowed disabled:opacity-60"
                        />

                        <button
                            type="button"
                            onClick={handleSkipToStep}
                            disabled={isSendingStepCommand}
                            className="mt-3 w-full rounded-xl bg-indigo-600 px-4 py-2.5 text-sm font-semibold text-white shadow-sm transition hover:bg-indigo-500 hover:shadow-md disabled:cursor-not-allowed disabled:opacity-60"
                        >
                            {isSendingStepCommand ? "Sending..." : "Go"}
                        </button>
                    </div>

                    <button
                        type="button"
                        onClick={handleForwardStep}
                        disabled={isSendingStepCommand}
                        className="flex h-full items-center justify-center gap-2 rounded-2xl border border-gray-200 hover:bg-blue-200 bg-white px-4 py-3 text-sm font-semibold text-gray-700 shadow-sm transition  hover:shadow-md disabled:cursor-not-allowed disabled:opacity-60"
                    >
                        Forward
                        <span className="text-lg">{">"}</span>
                    </button>
                </div>
            </div>

            <div className="mt-4 rounded-lg border p-4 shadow-sm">
                <div className="flex items-center justify-between">
                    <h2 className="text-lg font-semibold text-gray-900">Lesson Transcript</h2>
                </div>

                <div className="mt-2 h-[750px] overflow-y-auto rounded-2xl border bg-white p-4 space-y-4">
                    {conversation.length === 0 ? (
                        <p className="text-sm text-gray-500">No activity yet.</p>
                    ) : (
                        conversation.map((item) => renderMessage(item))
                    )}
                </div>

                <form onSubmit={addNote} className="mt-4 border-t pt-4">
                    <label className="text-sm font-medium text-gray-700">Add SLP Note</label>
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
    );
}