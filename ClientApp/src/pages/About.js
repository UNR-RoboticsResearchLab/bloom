export default function About() {
  
  return (
    <div className="min-h-screen bg-white text-gray-900 px-6 py-12 max-w-4xl mx-auto text-center" lang="en">

      {/* Header */}
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
        <h1 className="text-4xl font-bold mb-4">Bloom</h1>
        <p className="text-lg">CS 426 Senior Project in Computer Science, Spring 2026</p>
        <p className="text-lg">University of Nevada, Reno</p>
        <p className="text-lg">Department of Computer Science and Engineering</p>
      </div>

      {/* Team */}
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
        <h2 className="text-2xl font-semibold mb-3">Team</h2>
        <p><strong>Team Number</strong></p>
        <p>Team 24</p>
        <p className="mt-2"><strong>Project Name</strong></p>
        <p>Bloom</p>
        <p className="mt-2"><strong>Team Members</strong></p>
        <p>Illya Gavlovskyi, Max Knaefler, Jay Knight</p>
      </div>

      {/* Instructors & Advisors */}
      <div className="grid grid-cols-2 gap-6">
        <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
          <h2 className="text-2xl font-semibold mb-3">Instructors</h2>
          <p>David Feil-Seifer</p>
          <p>Vinh Le</p>
        </div>
        <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm">
          <h2 className="text-2xl font-semibold mb-3">Advisors</h2>
          <p>David Feil-Seifer (SARG)</p>
          <p>Denielle Oliva (SARG)</p>
        </div>
      </div>

      {/* Project Description */}
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm text-left">
        <h2 className="text-2xl font-semibold mb-3 text-center">Project Description</h2>
        <p>
          Bloom is an aid to Speech Language Pathologists to guide students that struggle in certain
          areas of language to aid them through speech therapy intervention. The overall goal is to have a
          system that is easy to use and provides students engagement and repetition with interventions.
          The system should be easy to set up, use, and be reliable in a real world setting.
        </p>
        <br />
        <h3 className="text-xl font-semibold mb-2">Intended Users</h3>
        <p>
          The intended user of the web app will be the Speech Language Pathologists and Teachers that will
          be assigning students their lessons based on the needs of the student. The other end of the table
          the user will be the student that is interacting with the robot and is going through the lesson.
          It is important to have an engaging experience to keep the student involved, interested and engaged.
        </p>
        <br />
        <h3 className="text-xl font-semibold mb-2">Functionality and Technology</h3>
        <p>
          The main functionality of Bloom includes interactive exercises that prompt students to speak and
          follow simple instructions. A robot guides the students, speaks to them, and reacts to their
          responses. The web interface allows Speech Language Pathology to assign activities, view performance
          data, and adjust difficulty levels. The system supports real time interaction, progress tracking,
          and personalized learning paths.
        </p>
        <br />
        <p>
          Bloom uses a web based platform with a frontend built using React and JavaScript. The backend is
          developed using .NET. Communication between the robot and the server is handled through secure APIs.
          Development tools include Docker for environment management, GitHub for version control, and VS Code
          for coding.
        </p>
        <br />
        <p>
          System hardware includes a small interactive robot equipped with speakers, microphones, and a display.
          It connects to the server over a network and executes the activities created in the web app.
        </p>
        <br />
        <h3 className="text-xl font-semibold mb-2">Reliability, Security, and Safety</h3>
        <p>
          Bloom is designed to be simple and reliable. Reliability is achieved through stable server architecture,
          error handling, and testing of all features. Security is ensured by user authentication, encrypted
          communication, and controlled access to data. Safety is addressed by following the guidelines and not
          storing any long term data. These precautions help ensure Bloom is safe and effective for real world use.
        </p>
      </div>

      {/* Project Related Resources */}
      <div className="mb-8 p-6 rounded-lg border border-gray-200 shadow-sm text-left">
        <h2 className="text-2xl font-semibold mb-6 text-center">Project Related Resources</h2>

        {/* Problem Domain Book */}
        <h3 className="text-xl font-semibold mb-3">Problem Domain Book</h3>
        <ul className="mb-6 space-y-2">
          <li>
            <a
              href="https://www.human-robot-interaction.org/"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              Human-Robot Interaction: An Introduction
            </a>
            <span className="text-gray-600"> — Bartneck, Bauer, Boekhorst, &amp; Kühnlenz (Cambridge University Press, 2020)</span>
          </li>
        </ul>

        {/* Related Websites */}
        <h3 className="text-xl font-semibold mb-3">Related Websites</h3>
        <ul className="mb-6 space-y-2">
          <li>
            <a href="https://www.asha.org/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              American Speech-Language-Hearing Association (ASHA)
            </a>
            <span className="text-gray-600"> — The national professional organization for speech-language pathologists</span>
          </li>
          <li>
            <a href="https://www.apraxia-kids.org/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              Apraxia Kids
            </a>
            <span className="text-gray-600"> — Resources and support for children with childhood apraxia of speech</span>
          </li>
          <li>
            <a href="https://www.hanen.org/home/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              The Hanen Centre
            </a>
            <span className="text-gray-600"> — Programs and resources for early language development in children</span>
          </li>
        </ul>

        {/* Technology Websites */}
        <h3 className="text-xl font-semibold mb-3">Technology Resources</h3>
        <ul className="mb-6 space-y-2">
          <li>
            <a href="https://react.dev/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              React
            </a>
            <span className="text-gray-600"> — Frontend JavaScript library used to build the Bloom web interface</span>
          </li>
          <li>
            <a href="https://learn.microsoft.com/dotnet/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              .NET Documentation
            </a>
            <span className="text-gray-600"> — Microsoft's framework used for Bloom's backend</span>
          </li>
          <li>
            <a href="https://docs.docker.com/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              Docker Documentation
            </a>
            <span className="text-gray-600"> — Containerization tool used for Bloom's development environment management</span>
          </li>
          <li>
            <a href="https://www.ros.org/" target="_blank" rel="noopener noreferrer" className="text-blue-600 underline hover:text-blue-800">
              Robot Operating System (ROS)
            </a>
            <span className="text-gray-600"> — Open-source robotics middleware used for robot communication</span>
          </li>
        </ul>

        {/* Academic Papers */}
        <h3 className="text-xl font-semibold mb-3">Technical Reports &amp; Journal Articles</h3>
        <ul className="mb-6 space-y-3">
          <li>
            <a
              href="https://www.mdpi.com/2071-1050/13/5/2771"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              A Case Study of a Robot-Assisted Speech Therapy for Children with Language Disorders
            </a>
          </li>
          <li>
            <a
              href="https://doi.org/10.1007/s12369-022-00946-2"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              Utilizing an Emotional Robot Capable of Lip-Syncing in Robot-Assisted Speech Therapy Sessions for Children with Language Disorders
            </a>
          </li>
          <li>
            <a
              href="https://doi.org/10.3390/s20226483"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              Integration of a Social Robot in a Pedagogical and Logopedic Intervention with Children: A Case Study
            </a>
          </li>
        </ul>

        {/* News */}
        <h3 className="text-xl font-semibold mb-3">News &amp; Current Events</h3>
        <ul className="space-y-3">
          <li>
            <a
              href="https://hai.stanford.edu/news/using-ai-to-streamline-speech-and-language-services-for-children"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              How AI Could Transform Speech Therapy for Children
            </a>
            <span className="text-gray-600"> — Stanford HAI, October 2025</span>
          </li>
          <li>
            <a
              href="https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12831455/"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              Social Robots in Speech Rehabilitation for Children: A Scoping Review
            </a>
            <span className="text-gray-600"> — PubMed, 2025</span>
          </li>
          <li>
            <a
              href="https://www.thebanner.com/education/early-childhood/speech-therapist-generative-ai-kids-DYAIDLCTWNC2NL7OA3MARI5EJM/"
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 underline hover:text-blue-800"
            >
              How Speech Therapists Are Using Generative AI to Help Their Kid Clients
            </a>
            <span className="text-gray-600"> — Baltimore Banner, November 2025</span>
          </li>
        </ul>
      </div>

    </div>
  );
}