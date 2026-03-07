import { useParams } from "react-router-dom";
import { useState } from "react";
import SelectStudentCard from "./SelectStudentCard";

export default function Lesson() {
    const { lessonName } = useParams();
    const [showSelectStudent, setShowSelectStudent] = useState(false);

    const students = [
    { id: 1, firstName: "John", lastName: "Doe", level: "Beginner" },
    { id: 2, firstName: "Jane", lastName: "Smith", level: "Intermediate" },
    { id: 3, firstName: "Alice", lastName: "Johnson", level: "Advanced" },
    ];

    function handleStudentSelect(student) {
        console.log("Selected student:", student);
        setShowSelectStudent(false);
    }

    return (
        <div>
            <p className=" text-sm font-semibold text-gray-900">
                Lessons &gt; {lessonName}
            </p>
            <div className= "rounded-lg border p-6 shadow-sm">
                <div className="text-center space-y-4">
                    <h1 className="text-4xl md:text-5xl font-semibold text-foreground leading-tight">
                        {lessonName}
                    </h1>
                </div>
                <div className="rounded-lg border p-4 shadow-sm flex items-center gap-4 justify-center">
                    <p className="text-sm font-semibold text-gray-900">
                        Time: 30 minutes
                    </p>
                    <p className="text-sm font-semibold text-gray-900">
                        Level: Beginner
                    </p>
                </div>
                <div className="mt-6 grid md:grid-cols-2 gap-6">
                    <div className="rounded-lg border p-4 shadow-sm">
                        <h2 className="text-lg font-semibold text-gray-900">Lesson Overview</h2>
                        <p className="mt-1 text-sm text-gray-500">
                            Description of the lesson goes here.
                        </p>
                    </div>
                    <div className= "rounded-lg border p-4 shadow-sm">
                        {/* <h2 className="text-lg font-semibold text-gray-900">Materials</h2>
                        <p className="mt-1 text-sm text-gray-500">
                            Here are the materials needed for this lesson.
                        </p> */}
                    </div>
                </div>
                {/* Start Lesson Button */}
                <div className="flex justify-center mt-8">
                    <button
                        onClick={() => setShowSelectStudent(true)}
                        className="rounded-md bg-indigo-600 px-6 py-3 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500"
                    >
                        Select Student to Start Lesson
                    </button>
                </div>
            </div>
            {/* Popup Modal */}
            {showSelectStudent && (
                <div className="fixed inset-0 flex items-center justify-center bg-black/40 z-50">
                    <SelectStudentCard
                        students={students}
                        onSelect={handleStudentSelect}
                        onCancel={() => setShowSelectStudent(false)}
                    />
                </div>
            )}
        </div>
         
    );
}