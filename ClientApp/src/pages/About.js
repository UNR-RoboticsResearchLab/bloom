

export default function About() {

  return (
    <div className="min-h-screen bg-white text-gray-900 px-6 py-12 max-w-4xl mx-auto text-center">
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
        <h1 className="text-4xl font-bold mb-4">
          Bloom
        </h1>
        <p className="text-lg">
          CS 426 Senior Project in Computer Science, Spring 2026
        </p>
        <p className="text-lg">
          University of Nevada, Reno
        </p>
        <p className="text-lg">
          Department of Computer Science and Engineering
        </p>   
      </div>
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
        <h2 className="text-2xl font-semibold mb-3">
          Team
        </h2>
        <p>
          <strong>Team Number</strong>
        </p>
        <p>
          Team 19
        </p>
        <p>
          <strong>Project Name</strong>
        </p>
        <p>
          Bloom
        </p>
        <p>
          <strong>Team Members</strong>
        </p>
        <p>
          Illya Gavlovskyi, Max Knaefler, Jay Knight
        </p>
      </div>
      <div className = "grid grid-cols-2 gap-6">
        <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
          <h2 className="text-2xl font-semibold mb-3">
            Instructors
          </h2>
          <p>
            <strong>Instructors Names</strong>
          </p>
          <p>
            David Feil-Seifer, Vinh Le
          </p>
        </div>
        <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
          <h2 className="text-2xl font-semibold mb-3">
            Advisors
          </h2>
          <p>
            <strong>Team Number:</strong>
          </p>
          <p>
            Team 24
          </p>
          <p>
            <strong>Advisor:</strong>
          </p>
          <p>
            David Feil-Seifer (SARG), Denielle Oliva (SARG)
          </p>
          <p>
            <strong>External Advisor:</strong>
          </p>
        </div>
      </div>
      
    </div>
  )
}

