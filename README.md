# bloom

Senior Projects 25-26 - Team 24

Bloom is an open source, dual-system platform for pediatric speech-language therapy. It pairs a
therapist-controlled web dashboard with a socially assistive robot (SAR): the SLP designs and
assigns the intervention, the robot delivers repetitive practice directly to the student with
speech and expressive facial feedback, and results are recorded automatically. The goal is
supervised autonomy — offloading the repetitive parts of a session to the robot while keeping the
clinician in control of the plan and pacing.

## Repository Structure

```
bloom/
|-- backend/     # .NET API - accounts, lessons, notes, RSR assessments
|-- frontend/    # React web dashboard for SLPs, teachers, and admins
|-- robot/       # Robot-side software: ROS2 stack + standalone conversation demo
`-- tests/       # Backend unit/integration tests (xUnit)
```

The robot stack (speech, face, lesson delivery) has its own setup docs — see
[robot/README.md](robot/README.md). Backend test details are in
[tests/README.md](tests/README.md).

## Setup Dev w/o Docker

1. Install Node.js, MySQL, dotnet core 9.0 SDK
2. Clone this repository
3. Run `npm install` in the project directory.
    You can also run `npm run build` and `npm start` to run the React frontend separately.
4. Create a MySQL database and update the connection string in `appsettings.json`
5. Run `dotnet run` in the project directory

## Setup Dev w/ Docker (Recommended)

1. Install Docker and Docker Compose
2. Clone this repository
3. Run `./startdocker-dev.sh` or `./startdocker-dev.bat` in the project directory (if it doesn't work, try `docker compose -f docker-compose.yml up --build` and/or `docker compose -f docker-compose.yml build --no-cache`)
4. Access the browser app at `http://localhost:3000` and the API at `http://localhost:5000`

To bring the physical robot (or the standalone conversation demo) into the loop, see
[robot/README.md](robot/README.md) — it runs separately from the web/backend stack above.

## API Documentation

[Swagger UI](https://localhost:5000/swagger/index.html) should launch after starting the server. If not, you can access it at `https://localhost:5000/swagger/index.html`.

## Testing

1. Run `dotnet test` in the project directory to execute unit tests.
