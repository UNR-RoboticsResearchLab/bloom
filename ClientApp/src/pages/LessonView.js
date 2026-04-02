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

    const [step, setStep] = useState([
        {
            id: 1,
            title: "Step 1: Introduction",
            text: "Hello. We are starting the lesson now.",
        },
        {
            id: 2,
            title: "Step 2: Practice",
            text: "Please say the word red.",
        },
        {
            id: 3,
            title: "Step 3: Review",
            text: "Let's review what we've learned.",
        }
    ]);


    // Fake data needs to be replaced with real conversation data from the backend
    const [conversation, setConversation] = useState([
        {
            id: 1,
            type: "robot",
            text: "Hello. We are starting the lesson now.",
            ts: new Date().toLocaleTimeString(),
        },
        {
            id: 2,
            type: "student",
            text: "Okay.",
            ts: new Date().toLocaleTimeString(),
        },
        {
            id: 3,
            type: "robot",
            text: "Please say the word red.",
            ts: new Date().toLocaleTimeString(),
        },
        {
            id: 4,
            type: "student",
            text: "Red.",
            ts: new Date().toLocaleTimeString(),
        },
        {
            id: 5,
            type: "note",
            text: "Student pronounced 'red' correctly but with a slight hesitation.",
            ts: new Date().toLocaleTimeString(),
        },
        {
            id: 6,
            type: "robot",
            text: "Let's review what we've learned.",
            ts: new Date().toLocaleTimeString(),
        }
    ]);

    // This effect starts the lesson. It checks if the lesson and student data are available, and if not, it redirects back to the lessons list. It also ensures that the lesson session is only started once using a ref.
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
            } catch (error) {
                console.error("Failed to start lesson session:", error);
            }
        }

        startLesson();
    }, [lesson, student, lessonId, sessionId, navigate, api]);

    // This function handles adding a new note to the conversation. It creates a new note object with a unique ID, the current timestamp, and the text from the input. The new note is then added to the conversation state, and the input field is cleared.
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

    // Handles going back one step in the lesson.
    function handleBackStep() {
        console.log("Back one step");
    }
    // Handles skipping forward one step in the lesson.
    function handleForwardStep() {
        console.log("Skip forward one step");
    }
    // Handles skipping to a specific step in the lesson based on user input.
    function handleSkipToStep() {
        const trimmed = stepInput.trim();
        if (!trimmed) return;
        console.log("Skip to step:", trimmed);
        setStepInput("");
    }

    // This function renders a single message in the conversation based on its type (robot, student, or note). It applies different styling for each type to visually distinguish them in the UI.
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
                        {new Date(item.ts).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}
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
                        {new Date(item.ts).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}
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
                    {new Date(item.ts).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}
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
                <h1 className="text-3xl font-semibold text-gray-900">
                    {lesson?.title ?? lesson?.Title}
                </h1>

                <p className="mt-2 text-sm text-gray-600">
                    {lesson?.description ?? lesson?.Description ?? "No description available."}
                </p>

                <div className="mt-4 grid gap-3 md:grid-cols-3 text-sm">
                    <div className="rounded-md border p-3">
                        <p className="font-semibold text-gray-900">Student</p>
                        <p className="mt-1 text-gray-700">
                            {student.firstName} {student.lastName}
                        </p>
                        <p className="text-gray-600">Level: {student.level}</p>
                    </div>

                    <div className="rounded-md border p-3">
                        <p className="font-semibold text-gray-900">Session</p>
                        <p className="mt-1 break-all text-gray-600">
                            {sessionId || "No paired session found"}
                        </p>
                    </div>

                    <div className="rounded-md border p-3">
                        <p className="font-semibold text-gray-900">Lesson ID</p>
                        <p className="mt-1 break-all text-gray-600">
                            {lessonId}
                        </p>
                    </div>
                </div>
            </div>



            <div className="mt-6 rounded-lg border p-4 shadow-sm">
                <div className="flex items-center justify-between">
                    <h2 className="text-lg font-semibold text-gray-900">Lesson Controls</h2>
                </div>

                <div className="mt-4 h-[250px] overflow-x-auto overflow-y-hidden rounded-2xl border bg-white p-4">
                    <div className="flex gap-4 h-full items-stretch">
                        {step.map((item) => (
                            <div
                                key={item.id}
                                className="min-w-[250px] flex-shrink-0 h-full rounded-xl bg-cyan-500 border border-gray-300 p-4 shadow-sm flex flex-col"
                            >
                                <h4 className="font-semibold text-white">{item.title}</h4>
                                <p className="mt-1 text-white">{item.text}</p>
                            </div>
                        ))}
                    </div>
                </div>

                <div className="mt-4 grid gap-4 md:grid-cols-[1fr_2fr_1fr] items-stretch">
    
                    {/* Back Button */}
                    <button
                        type="button"
                        onClick={handleBackStep}
                        className="flex h-full items-center justify-center gap-2 rounded-2xl border border-gray-200 bg-white px-4 py-3 text-sm font-semibold text-gray-700 shadow-sm transition hover:bg-gray-100 hover:shadow-md"                    >
                        <span className="text-lg">{"<"}</span>
                        Back
                    </button>

                    {/* Middle Control */}
                    <div className="flex flex-col items-center rounded-2xl border border-gray-200 bg-white p-4 shadow-sm">
                        <p className="mb-2 text-xs font-semibold text-gray-500 uppercase tracking-wide">
                            Jump to Step
                        </p>

                        <input
                            type="number"
                            value={stepInput}
                            onChange={(e) => setStepInput(e.target.value)}
                            placeholder="Step #"
                            className="w-full rounded-xl border border-gray-300 p-3 text-center text-sm font-medium shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
                        />

                        <button
                            type="button"
                            onClick={handleSkipToStep}
                            className="mt-3 w-full rounded-xl bg-indigo-600 px-4 py-2.5 text-sm font-semibold text-white shadow-sm transition hover:bg-indigo-500 hover:shadow-md"
                        >
                            Go
                        </button>
                    </div>

                    {/* Forward Button */}
                    <button
                        type="button"
                        onClick={handleForwardStep}
                        className="flex h-full items-center justify-center gap-2 rounded-2xl border border-gray-200 bg-white px-4 py-3 text-sm font-semibold text-gray-700 shadow-sm transition hover:bg-gray-100 hover:shadow-md"
                    >
                        Forward
                        <span className="text-lg">{">"}</span>
                    </button>
                    
                </div>
            </div>





            <div className="mt-6 rounded-lg border p-4 shadow-sm">
                <div className="flex items-center justify-between">
                    <h2 className="text-lg font-semibold text-gray-900">Lesson Transcript</h2>
                </div>

                <div className="mt-4 h-[500px] overflow-y-auto rounded-2xl border bg-white p-4 space-y-4">
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