# bloom

Senior Projects 25-26 - Team 24

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

## API Documentation

[Swagger UI](https://localhost:5000/swagger/index.html) should launch after starting the server. If not, you can access it at `https://localhost:5000/swagger/index.html`.

## Testing

1. Run `dotnet test` in the project directory to execute unit tests.

