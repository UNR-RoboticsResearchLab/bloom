import { useParams } from "react-router-dom";

export default function Lesson() {
    const { lessonName } = useParams();
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
            </div>
        </div>
         
    );
}