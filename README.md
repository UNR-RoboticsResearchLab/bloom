# bloom

Senor Project Team 25

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
3. Run `docker-compose up --build` in the project directory
4. Access the application at `http://localhost:5000`
5. Migrate the database
   - Open a new terminal window
   - Run `docker ps` to get the container ID of the running application
   - Run `docker exec -it server-dev dotnet ef database update` to apply migrations

## API Documentation

[Future Link to API Documentation Here]

## Testing

1. Run `dotnet test` in the project directory to execute unit tests.

