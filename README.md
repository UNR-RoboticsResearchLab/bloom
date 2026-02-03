# bloom

Senor Project Team 24

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
3. Run `./startserver-dev.sh` or `./startserver-dev.bat` in the project directory. The script handles launching all components ( frontend, backend, database ) for development purposes.
4. Access the API at `http://localhost:5000`, and the browser app at `http://localhost:3000`

## Project Structure

/bloom/
|-- ClientApp: React Frontend
|-- Controllers: The controllers that map to endpoints in the REST API
|-- Data: Database middleware configuration
|-- Migrations: Database migrations
|-- Models: The primary data types in use by the API
|-- Repositories: Abstraction Layer for using data stores
|-- Robot: ROS drivers, face, speech and listening
|-- Services: Business-level Abstraction layer for data stores
|-- Program.cs: API Entrypoint


## API Documentation

https://localhost:5000/swagger

## Testing

1. Run `dotnet test` in the project directory to execute unit tests.

