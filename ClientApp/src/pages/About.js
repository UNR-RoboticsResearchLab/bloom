

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
          Team 24
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
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
        <h2 className="text-2xl font-semibold mb-3">
          Project Description
        </h2>
        <p>
          Bloom is an aid to Speech Language Pathologists to guide students that struggle with certain areas of language to go through speech therapy routines. The overall goal is to have a system that is easy to use and provides students engagement and repetition with the lessons. The system should be easy to launch, get to where you need to go, have all information needed displayed intuitively and fast.
        </p>
        <p>
          The intended user of the web app will be the Speech Language Pathologists and Teachers that will be assigning students their lessons based on the needs of the student. The other end of the table the user will be the student that is interacting with the robot and is going through the lesson. It is important to have the most engaging experience to keep the student involved.

        </p>
        <p>
          The main functionality of Bloom includes interactive exercises that prompt students to speak and follow simple instructions. A robot guides the students, speaks to them, and reacts to their responses. The web interface allows Speech Language Pathology to assign activities, view performance data, and adjust difficulty levels. The system supports real time interaction, progress tracking, and personalized learning paths.

        </p>
        <p>
          Bloom uses a web based platform with a frontend built using React and JavaScript. The backend is developed using .NET. Communication between the robot and the server is handled through secure APIs.  Development tools include Docker for environment management, GitHub for version control, and VS code for coding. 
        </p>
        <p>
          System hardware includes a small interactive robot equipped with microphones, and a display. It connects to the server over a network and executes the activities created in the web app.

        </p>
        <p>
          Bloom is designed to be simple and reliable. Reliability is achieved through stable server architecture, error handling, and testing of all features. Security is ensured by user authentication, encrypted communication, and controlled access to data. Safety is addressed by following the guidelines and not storing any long term data. These prosecutions help ensure Bloom is safe and effective for real world use.
        </p>
      </div>
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
        <h2 className="text-2xl font-semibold mb-3">
          Project Related Resources
        </h2>
        <p>
          <strong>A Case Study of a Robot-Assisted Speech Therapy for Children with Language Disorders</strong>
        </p>
        <p>
          https://www.mdpi.com/2071-1050/13/5/2771
        </p>
        <p>
          <strong>Utilizing an Emotional Robot Capable of Lip-Syncing in Robot-Assisted Speech Therapy Sessions for Children with Language Disorders</strong>
        </p>
        <p>
          https://doi.org/10.1007/s12369-022-00946-2
        </p>
        <p>
          <strong>Integration of a Social Robot in a Pedagogical and Logopedic Intervention with Children: A Case Study</strong>
        </p>
        <p>
          https://doi.org/10.3390/s20226483
        </p> 
      </div>
      
    </div>
  )
}

